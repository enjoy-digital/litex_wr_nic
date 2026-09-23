#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""White Rabbit on the Tang Mega 138K Pro's hardened AE350 CPU.

The GW5AST-138B carries a hardened Andes AE350 platform (A25, 32-bit
RISC-V). This target runs the WRPC firmware on it instead of the soft uRV:

  - the CPU's fixed reset address (0x80000000) holds a two-instruction stub
    that jumps to 0x00000000,
  - the fabric memory at 0x00000000 holds the WRPC image, so WRPC keeps its
    own linker script and 128 KiB layout,
  - WRPC's peripheral window is mapped at 0xe9000000, inside the CPU's
    uncached peripheral (EXTS AHB) range; WR decodes address bits 15:2 only,
    so the firmware's peripheral base is simply that address,
  - the SoftPLL interrupt drives the CPU's first user interrupt input, a
    source of its own PLIC, which the firmware claims and completes.

The gateware is the same as tang_mega_138k_pro_wr.py otherwise: LiteEth's
raw SerDes with WR's PCS, the MS5351 and GW5A PLL clock actuators, UARTBone
on the physical UART and WRPC's console on the CSR-backed crossover.

See doc/tang_mega_138k_pro.md; this target is not timing-clean yet.
"""

import sys
import argparse
import subprocess

from pathlib import Path

from migen import *
from migen.fhdl.specials import Tristate

from litex.gen import *

from litex.build.generic_platform import IOStandard, Pins, Subsignal

from litex.soc.interconnect.csr import CSRField, CSRStatus, CSRStorage

from litex.soc.cores.bitbang import I2CMaster
from litex.soc.cores.clock.gowin_gw5a import GW5APLL
from litex.soc.cores.freqmeter import FreqMeter
from litex.soc.cores.uart import UARTPHY, UART
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.integration.common import get_mem_data

from litex_boards.platforms import sipeed_tang_mega_138k_pro

from litex_wr_nic.gateware.wr_clock import WRGowinPLLBackend, WRMS5351Backend
from litex_wr_nic.gateware.wr_cpu   import wr_cpu_firmware_filename
from litex_wr_nic.gateware.wr_core  import add_white_rabbit
from litex_wr_nic.gateware.wr_phy   import GW5WRPHY

# Constants ----------------------------------------------------------------------------------------

FIRMWARE_DIR         = Path(__file__).resolve().parent / "litex_wr_nic/firmware"

WR_PERIPHERAL_ORIGIN = 0xe900_0000 # Firmware DEV_BASE; inside the CPU's peripheral range.
WR_PERIPHERAL_SIZE   = 0x0001_0000 # WR decodes address bits 15:2.
FIRMWARE_ORIGIN      = 0x0000_0000 # WRPC's own linker script.
FIRMWARE_SIZE        = 128*1024

# IOs ----------------------------------------------------------------------------------------------

# Pro dock I2C: the MS5351 clock generator and the SFP EEPROMs share one bus
# behind a 74HC4051 multiplexer, as on the uRV target. Channel 3 is the MS5351
# feeding Q1 REFCLK1 (PLL0).
_dock_i2c_io = [
    ("dock_i2c", 0,
        Subsignal("scl", Pins("K25")),
        Subsignal("sda", Pins("K26")),
        IOStandard("LVCMOS33")
    ),
    ("dock_i2c_sel", 0, Pins("N19 P19 P26"), IOStandard("LVCMOS33")),
]
DOCK_I2C_CHANNEL_MS5351 = 3

# The CPU's reset address is fixed; jump to the firmware: lui t0, 0; jalr x0, 0(t0).
BOOT_STUB = [0x0000_02b7, 0x0002_8067]

# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform, cpu_clk_freq=750e6):
        self.cd_sys     = ClockDomain()
        self.cd_cpu     = ClockDomain()
        self.cd_wr_dmtd = ClockDomain()

        # # #

        # One PLL: a 750 MHz VCO divides to the CPU clock and to 62.5 MHz for
        # the WR system and DDMTD clocks. The AE350 needs its clock from the
        # PLL at PLL_R[0], so a single PLL keeps that constraint unambiguous.
        self.pll = pll = GW5APLL(device=platform.device, devicename=platform.devicename)
        pll.vco_freq_range = (650e6, 1300e6)
        pll.register_clkin(platform.request("clk50"), 50e6)
        pll.create_clkout(self.cd_cpu,     cpu_clk_freq, margin=0, with_reset=False)
        pll.create_clkout(self.cd_sys,     62.5e6, margin=0)
        pll.create_clkout(self.cd_wr_dmtd, 62.5e6, margin=0)
        # The helper actuator steps the DDMTD output's phase; the CPU and
        # system outputs of the same PLL are unaffected.
        pll.expose_dpa(clkout=2)
        platform.toolchain.additional_cst_commands.append('INS_LOC "PLL" PLL_R[0]')

# WR AE350 SoC -------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def add_csr_bridge(self, *args, **kwargs):
        # LiteX registers the CSR bridge only for SoCs with SDRAM. The shared
        # read path, from the bus arbiter's grant through every bank, does not
        # meet 62.5 MHz here without it.
        kwargs["with_register"] = True
        return super().add_csr_bridge(*args, **kwargs)

    def __init__(self, sfp=0,
        cpu_clk_freq = 750e6,
        cpu_firmware = None,
        main_center  = 810000,
        main_shift   = 1,
    ):
        # The CPU runs the image from its own memory; the core's private
        # dual-port memory is left out, so its initialisation is unused.
        cpu_firmware = cpu_firmware or str(FIRMWARE_DIR /
            wr_cpu_firmware_filename("external", "bin", "tang_mega_138k_pro"))
        dpram_initf  = str(Path(cpu_firmware).with_suffix(".bram"))
        platform = sipeed_tang_mega_138k_pro.Platform()
        platform.add_extension(_dock_i2c_io)
        platform.toolchain.options["maxfan"]       = 32
        platform.toolchain.options["place_option"] = 3
        platform.toolchain.options["route_option"] = 2
        self.crg = CRG(platform, cpu_clk_freq)

        SoCCore.__init__(self, platform, 62.5e6,
            cpu_type             = "gowin_ae350",
            ident                = "LiteX WR on Tang Mega 138K Pro (AE350)",
            # Boot stub at the CPU's fixed reset address.
            integrated_rom_size  = 0x1000,
            integrated_rom_init  = BOOT_STUB,
            # The WRPC image, in the CPU's memory region.
            integrated_sram_size = FIRMWARE_SIZE,
            integrated_sram_init = get_mem_data(cpu_firmware, endianness="little", mem_size=FIRMWARE_SIZE),
            with_uart            = False,
            with_timer           = False,
        )
        # WRPC's linker script places the image at 0, where the CPU's memory
        # map puts its fabric memory.
        assert self.bus.regions["sram"].origin == FIRMWARE_ORIGIN
        self.add_config("CPU_CLK_FREQ", int(cpu_clk_freq))

        # UARTBone ---------------------------------------------------------------------------------
        self.add_uartbone(baudrate=115200)
        console_pads = Record([("tx", 1), ("rx", 1)])
        self.uart_xover_phy = UARTPHY(console_pads, 62.5e6, baudrate=115200)
        self.uart_xover = UART(self.uart_xover_phy,
            tx_fifo_depth = 128,
            rx_fifo_depth = 4096,
            rx_fifo_rx_we = True,
        )
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
        wr = add_white_rabbit(self,
            cpu_firmware     = dpram_initf,
            cpu_type         = "external",
            board_name       = "T138",
            phy              = self.phy,
            with_ext_clk     = False,
            serial_pads      = wr_serial,
            sfp_los_pads     = pads.los,
            sfp_disable_pads = pads.tx_disable,
        )
        self.comb += [
            wr.source.ready.eq(1),
            wr.sink.valid.eq(0),
        ]
        # WRPC's peripheral window for the hard CPU.
        self.bus.add_slave(name="wr_cpu_periph", slave=wr.cpu_peripheral_bus,
            region=SoCRegion(origin=WR_PERIPHERAL_ORIGIN, size=WR_PERIPHERAL_SIZE, cached=False))
        # The SoftPLL interrupt drives the CPU's first user interrupt input;
        # the firmware enables that PLIC source and completes every claim.
        assert not self.irq.locs, "GP_INT[0] is reserved for the WR SoftPLL."
        self.comb += self.cpu.interrupt[0].eq(wr.cpu_irq)

        # Clock actuators --------------------------------------------------------------------------
        self.helper_tuning = helper_tuning = WRGowinPLLBackend(cd="sys")
        self.comb += [
            helper_tuning.command.load.eq(wr.dac_dmtd_load),
            helper_tuning.command.data.eq(wr.dac_dmtd_data),
            self.crg.pll.phase_dir.eq(helper_tuning.phase_dir),
            self.crg.pll.phase_step.eq(helper_tuning.phase_step),
        ]
        self.main_tuning = main_tuning = WRMS5351Backend(62.5e6,
            center     = main_center,
            shift      = main_shift,
            multiplier = 36,
            pll        = "A",
        )
        self.comb += [
            main_tuning.command.load.eq(wr.dac_refclk_load),
            main_tuning.command.data.eq(wr.dac_refclk_data),
        ]

        # Dock I2C ---------------------------------------------------------------------------------
        self.i2c     = I2CMaster(connect_pads=False)
        self.i2c_sel = CSRStorage(3, reset=DOCK_I2C_CHANNEL_MS5351,
            description="Dock I2C multiplexer channel for host accesses.")
        i2c_pads = platform.request("dock_i2c")
        tuner    = main_tuning.tuner
        scl_i    = Signal()
        sda_i    = Signal()
        self.specials += [
            Tristate(i2c_pads.scl, o=0, i=scl_i,
                oe=Mux(tuner.enable, ~tuner.scl_o, ~self.i2c._w.fields.scl)),
            Tristate(i2c_pads.sda, o=0, i=sda_i,
                oe=Mux(tuner.enable, ~tuner.sda_o,
                    self.i2c._w.fields.oe & ~self.i2c._w.fields.sda)),
        ]
        self.comb += [
            tuner.sda_i.eq(sda_i),
            self.i2c._r.fields.scl.eq(scl_i),
            self.i2c._r.fields.sda.eq(sda_i),
            platform.request("dock_i2c_sel").eq(
                Mux(tuner.enable, DOCK_I2C_CHANNEL_MS5351, self.i2c_sel.storage)),
        ]

        # Diagnostics ------------------------------------------------------------------------------
        self.wr_cpu_status = CSRStatus(fields=[
            CSRField("irq",   description="WRPC SoftPLL interrupt request."),
            CSRField("reset", description="WRPC's reset request for the external CPU."),
        ])
        self.comb += [
            self.wr_cpu_status.fields.irq.eq(wr.cpu_irq),
            self.wr_cpu_status.fields.reset.eq(wr.cpu_reset),
        ]
        self.ref_clk_freq  = FreqMeter(62_500_000, clk=ClockSignal("wr"))
        self.dmtd_clk_freq = FreqMeter(62_500_000, clk=ClockSignal("wr_dmtd"))
        self.rx_clk_freq   = FreqMeter(62_500_000, clk=self.phy.rx_clk)

        # LEDs -------------------------------------------------------------------------------------
        self.comb += [
            platform.request("led_n", 0).eq(~wr.led_link),
            platform.request("led_n", 1).eq(~wr.led_act),
            platform.request("led_n", 2).eq(~wr.led_pps),
        ]

        # Timing constraints -----------------------------------------------------------------------
        sys_clk  = self.crg.pll.clkouts[1].clk
        dmtd_clk = self.crg.pll.clkouts[2].clk
        platform.add_generated_clock_constraint(sys_clk, platform.lookup_request("clk50"),
            divide_by=4, multiply_by=5, name="sys")
        platform.add_generated_clock_constraint(dmtd_clk, platform.lookup_request("clk50"),
            divide_by=4, multiply_by=5, name="dmtd")
        platform.add_period_constraint(self.phy.tx_clk, 8)
        platform.add_period_constraint(self.phy.rx_clk, 8)
        platform.add_false_path_constraints(
            sys_clk, dmtd_clk, self.phy.tx_clk, self.phy.rx_clk)


def main():
    parser = argparse.ArgumentParser(description="White Rabbit on Tang Mega 138K Pro's AE350 CPU.")

    # Build/load options.
    parser.add_argument("--build",               action="store_true", help="Build the bitstream.")
    parser.add_argument("--load",                action="store_true", help="Load the bitstream.")
    parser.add_argument("--skip-firmware-build", action="store_true", help="Reuse the existing WR firmware.")
    parser.add_argument("--output-dir", default="build/tang_mega_138k_pro_wr_ae350", help="Build output directory.")

    # PHY/CPU options.
    parser.add_argument("--sfp", type=int, choices=(0, 1), default=0, help="SFP lane (default: 0).")
    parser.add_argument("--cpu-clk-freq", type=float, default=750e6,
        help="AE350 core clock frequency (default: 750 MHz).")
    parser.add_argument("--main-center", type=lambda v: int(v, 0), default=810000,
        help="MS5351 code for a neutral main-clock command (default: 810000, bench-calibrated).")
    parser.add_argument("--main-shift", type=int, default=1,
        help="MS5351 codes per main-clock command step, as a power of two (default: 1).")
    args = parser.parse_args()

    # Firmware: the AE350 profile relocates WRPC's peripheral window and
    # services the SoftPLL interrupt through the CPU's PLIC.
    if not args.skip_firmware_build:
        subprocess.run([
            sys.executable, str(FIRMWARE_DIR / "build.py"),
            "--target", "tang_mega_138k_pro", "--read-only-storage",
            "--wr-cpu-type", "external", "--peripheral-origin", hex(WR_PERIPHERAL_ORIGIN),
        ], check=True)

    # Gateware.
    soc = BaseSoC(
        sfp          = args.sfp,
        cpu_clk_freq = args.cpu_clk_freq,
        main_center  = args.main_center,
        main_shift   = args.main_shift,
    )
    builder = Builder(soc,
        output_dir       = args.output_dir,
        csr_csv          = str(Path(args.output_dir) / "csr.csv"),
        # The CPU runs WRPC, not the LiteX BIOS.
        compile_software = False,
    )
    builder.build(run=args.build)
    if args.load:
        soc.platform.create_programmer().load_bitstream(builder.get_bitstream_filename(mode="sram"))


if __name__ == "__main__":
    main()
