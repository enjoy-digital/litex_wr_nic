#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys
import argparse
import subprocess

from pathlib import Path

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import CSRStatus

from litex.soc.cores.clock.gowin_gw5a import GW5APLL
from litex.soc.cores.freqmeter import FreqMeter
from litex.soc.cores.uart import UARTPHY, UART
from litex.soc.integration.soc_core import SoCMini
from litex.soc.integration.builder import Builder
from litex.soc.integration.common import get_mem_data

from litex_boards.platforms import sipeed_tang_mega_138k_pro

from litex_wr_nic.gateware.wr_core import add_white_rabbit
from litex_wr_nic.gateware.wr_phy  import GW5WRPHY

# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform):
        self.cd_sys     = ClockDomain()
        self.cd_wr_dmtd = ClockDomain()

        # # #

        self.pll = pll = GW5APLL(device=platform.device, devicename=platform.devicename)
        pll.register_clkin(platform.request("clk50"), 50e6)
        pll.create_clkout(self.cd_sys,     62.5e6, margin=0)
        # The 8-bit WR core divides the 125 MHz inputs by two for DDMTD.
        pll.create_clkout(self.cd_wr_dmtd, 62.5e6, margin=0)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCMini):
    def __init__(self, sfp=0,
        with_analyzer = False,
        analyzer_csv  = "analyzer.csv",
        cpu_firmware  = "litex_wr_nic/firmware/tang_mega_138k_pro_wrc.bram",
    ):
        platform = sipeed_tang_mega_138k_pro.Platform()
        # Limit reset/control fanout in the 125 MHz WR logic.
        platform.toolchain.options["maxfan"] = 32
        self.crg = CRG(platform)
        SoCMini.__init__(self, platform, 62.5e6, ident="LiteX WR bring-up on Tang Mega 138K Pro")

        # UART -------------------------------------------------------------------------------------
        # GW5 has no supported LiteX JTAGBone primitive yet. Share the USB
        # UART transport between host diagnostics and a CSR-backed WR console.
        self.add_uartbone(baudrate=115200)
        console_pads = Record([("tx", 1), ("rx", 1)])
        self.uart_xover_phy = UARTPHY(console_pads, 62.5e6, baudrate=115200)
        self.uart_xover = UART(self.uart_xover_phy,
            tx_fifo_depth = 128,
            rx_fifo_depth = 4096,
            rx_fifo_rx_we = True,
        )
        # Buffer complete console replies and allow burst reads over UARTBone.
        self.console_rx_level = CSRStatus(len(self.uart_xover.rx_fifo.level),
            description="Number of WR console bytes available for host reads.")
        self.comb += self.console_rx_level.status.eq(self.uart_xover.rx_fifo.level)
        wr_serial = Record([("tx", 1), ("rx", 1)])
        self.comb += [
            wr_serial.rx.eq(console_pads.tx),
            console_pads.rx.eq(wr_serial.tx),
        ]

        # White Rabbit -----------------------------------------------------------------------------
        self.phy = GW5WRPHY(platform, lane=sfp)
        pads = platform.request("sfp", sfp)
        # Reuse LiteX RAM and the existing WR CPU memory bridge. This avoids
        # vendor-specific byte-write RAM inference in the VHDL CPU wrapper.
        self.add_ram("wr_cpu_ram",
            origin   = 0x1000_0000,
            size     = 128*1024,
            contents = get_mem_data(str(Path(cpu_firmware).with_suffix(".bin")),
                endianness = "little",
                mem_size   = 128*1024,
            ),
        )
        wr = add_white_rabbit(self,
            cpu_firmware      = cpu_firmware,
            cpu_memory_region = self.bus.regions["wr_cpu_ram"],
            board_name        = "T138",
            phy               = self.phy,
            with_ext_clk      = False,
            serial_pads       = wr_serial,
            sfp_los_pads      = pads.los,
            sfp_disable_pads  = pads.tx_disable,
        )
        self.comb += [
            wr.source.ready.eq(1),
            wr.sink.valid.eq(0),
        ]

        # Diagnostics ------------------------------------------------------------------------------
        # Keep the unconnected clock-actuator commands visible for bench development.
        self.main_dac   = CSRStatus(16, description="Last WR main-clock command; no actuator connected yet.")
        self.helper_dac = CSRStatus(16, description="Last WR helper-clock command; no actuator connected yet.")
        self.sync += [
            If(wr.dac_refclk_load, self.main_dac.status.eq(wr.dac_refclk_data)),
            If(wr.dac_dmtd_load,   self.helper_dac.status.eq(wr.dac_dmtd_data)),
        ]
        self.ref_clk_freq  = FreqMeter(62_500_000, clk=ClockSignal("wr"))
        self.dmtd_clk_freq = FreqMeter(62_500_000, clk=ClockSignal("wr_dmtd"))
        self.rx_clk_freq   = FreqMeter(62_500_000, clk=self.phy.rx_clk)
        if with_analyzer:
            from litescope import LiteScopeAnalyzer
            platform.toolchain.options["place_option"] = 2
            platform.toolchain.options["route_option"] = 1
            rx_raw = Signal(10)
            self.comb += rx_raw.eq(self.phy.serdes.rx_data[:10])
            # Narrow groups and a block-RAM-sized trigger FIFO keep the
            # compare/consume/read path short enough for the recovered clock.
            self.analyzer = LiteScopeAnalyzer(
                groups = {
                    0: [rx_raw, self.phy.serdes.rx_valid,
                        self.phy.serdes.rx_empty, self.phy.serdes.aligned],
                    1: [self.phy.rx_data, self.phy.rx_k, self.phy.rx_error, self.phy.ready],
                },
                depth            = 1024,
                samplerate       = 125e6,
                clock_domain     = "wr_phy_rx",
                trigger_depth    = 128,
                subsampler_width = 4,
                csr_csv          = analyzer_csv,
            )

        # LEDs -------------------------------------------------------------------------------------
        self.comb += [
            platform.request("led_n", 0).eq(~wr.led_link),
            platform.request("led_n", 1).eq(~wr.led_act),
            platform.request("led_n", 2).eq(~wr.led_pps),
        ]

        # Timing constraints -----------------------------------------------------------------------
        # Constrain the PLL output nets: Gowin removes the clock-domain aliases.
        sys_clk  = self.crg.pll.clkouts[0].clk
        dmtd_clk = self.crg.pll.clkouts[1].clk
        platform.add_generated_clock_constraint(sys_clk, platform.lookup_request("clk50"),
            divide_by=4, multiply_by=5, name="sys")
        platform.add_generated_clock_constraint(dmtd_clk, platform.lookup_request("clk50"),
            divide_by=4, multiply_by=5, name="dmtd")
        platform.add_period_constraint(self.phy.tx_clk, 8)
        platform.add_period_constraint(self.phy.rx_clk, 8)
        platform.add_false_path_constraints(
            sys_clk, dmtd_clk, self.phy.tx_clk, self.phy.rx_clk)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Initial WR PHY/console bring-up on Tang Mega 138K Pro.")

    # Build/load options.
    parser.add_argument("--build",               action="store_true", help="Build the bitstream.")
    parser.add_argument("--load",                action="store_true", help="Load the bitstream.")
    parser.add_argument("--skip-firmware-build", action="store_true", help="Reuse the existing WR firmware.")
    parser.add_argument("--output-dir", default="build/tang_mega_138k_pro_wr", help="Build output directory.")

    # PHY/debug options.
    parser.add_argument("--sfp", type=int, choices=(0, 1), default=0, help="SFP lane (default: 0).")
    parser.add_argument("--with-analyzer", action="store_true",
        help="Capture raw and decoded RX symbols with LiteScope.")
    args = parser.parse_args()

    # Firmware.
    if not args.skip_firmware_build:
        subprocess.run([
            sys.executable, "litex_wr_nic/firmware/build.py",
            "--target", "tang_mega_138k_pro", "--read-only-storage",
        ], check=True)

    # Gateware.
    soc = BaseSoC(
        sfp           = args.sfp,
        with_analyzer = args.with_analyzer,
        analyzer_csv  = str(Path(args.output_dir) / "analyzer.csv"),
    )
    builder = Builder(soc,
        output_dir = args.output_dir,
        csr_csv    = str(Path(args.output_dir) / "csr.csv"),
    )
    builder.build(run=args.build)
    if args.load:
        soc.platform.create_programmer().load_bitstream(builder.get_bitstream_filename(mode="sram"))


if __name__ == "__main__":
    main()
