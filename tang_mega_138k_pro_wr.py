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
from migen.fhdl.specials import Tristate

from litex.gen import *

from litex.build.generic_platform import IOStandard, Misc, Pins, Subsignal

from litex.soc.interconnect.csr import CSRField, CSRStatus, CSRStorage

from litex.soc.cores.bitbang import I2CMaster
from litex.soc.cores.clock.gowin_gw5a import GW5APLL
from litex.soc.cores.freqmeter import FreqMeter
from litex.soc.cores.uart import UARTPHY, UART
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.soc_core import SoCMini
from litex.soc.integration.builder import Builder
from litex.soc.integration.common import get_mem_data

from litex_boards.platforms import sipeed_tang_mega_138k_pro

from litex_wr_nic.gateware.pps             import PPSGenerator
from litex_wr_nic.gateware.sfp_eeprom      import SFPEEPROMCache
from litex_wr_nic.gateware.pps_timestamper import GW5OversampledInput, PPSTimestamper
from litex_wr_nic.gateware.wr_clock        import WRGowinPLLBackend, WRMS5351Backend
from litex_wr_nic.gateware.wr_core         import add_white_rabbit
from litex_wr_nic.gateware.wr_cpu          import (
    WR_CPU_MEMORY_ORIGIN,
    WR_CPU_MEMORY_SIZE,
    WR_CPU_PROFILES,
    resolve_wr_cpu_variant,
    wr_cpu_firmware_filename,
)
from litex_wr_nic.gateware.wr_phy          import GW5WRPHY

# IOs ----------------------------------------------------------------------------------------------

# Pro dock I2C: the FPGA SYS_TWI bus reaches the SFP EEPROMs and the MS5351
# clock generators through a 74HC4051 multiplexer. Channel 3 is the MS5351
# feeding Q1 REFCLK1 (PLL0); channels 0 and 1 are the SFP0/SFP1 modules.
_dock_i2c_io = [
    ("dock_i2c", 0,
        Subsignal("scl", Pins("K25")),
        Subsignal("sda", Pins("K26")),
        IOStandard("LVCMOS33")
    ),
    ("dock_i2c_sel", 0, Pins("N19 P19 P26"), IOStandard("LVCMOS33")),
]

# Pro dock PMOD0 (2x6 header, 3.3 V): IO0 on pin 1 and IO2 on pin 3, per the
# dock schematic. The WR PPS is output on IO2; an external PPS is timestamped
# on IO0 for independent alignment measurements.
_pmod0_pps_io = [
    # The input is pulled down so a floating header reads a stable low.
    ("pps_in",  0, Pins("N18"), IOStandard("LVCMOS33"), Misc("PULL_MODE=DOWN")),
    ("pps_out", 0, Pins("R16"), IOStandard("LVCMOS33"), Misc("DRIVE=8")),
]

DOCK_I2C_CHANNEL_MS5351 = 3
DOCK_I2C_CHANNEL_SFP    = 0 # SFP0; SFP1 is channel 1.

# Constants ----------------------------------------------------------------------------------------

FIRMWARE_DIR  = Path(__file__).resolve().parent / "litex_wr_nic/firmware"
FIRMWARE_SIZE = 128*1024

# How the WR core is configured for each firmware profile this board offers,
# and how each CPU is spelled. The GW5AST-138B's AE350 is a hard CPU, so the
# SoC owns it and the core is configured "external": it then exposes WRPC's
# peripheral window, interrupt and reset request for the SoC to connect.
WR_CPU_CORE_TYPES = {"urv": "urv", "vexriscv": "vexriscv", "ae350": "external"}
WR_CPU_NAMES      = {"urv": "uRV", "vexriscv": "VexRiscv", "ae350": "AE350"}

# The uRV runs from the core's own single-cycle RAM, which the host reaches
# for firmware loading and debug; a LiteX CPU runs from SoC RAM, as on the
# other boards.
WR_CPU_LOCAL_MEMORY_ORIGIN = 0x1000_0000

# The AE350's image lives in fabric memory at the address WRPC links for, and
# WRPC's registers in the CPU's uncached peripheral range. WR decodes address
# bits 15:2 of that window, so only the base matters.
AE350_CLK_FREQ          = 750e6
AE350_FIRMWARE_ORIGIN   = 0x0000_0000
AE350_PERIPHERAL_ORIGIN = 0xe900_0000
AE350_PERIPHERAL_SIZE   = 0x0001_0000
# The CPU's reset address is fixed; jump to the firmware: lui t0, 0; jalr x0, 0(t0).
AE350_BOOT_STUB = [0x0000_02b7, 0x0002_8067]

# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform, cpu_clk_freq=None):
        self.cd_sys     = ClockDomain()
        self.cd_wr_dmtd = ClockDomain()

        # # #

        self.pll = pll = GW5APLL(device=platform.device, devicename=platform.devicename)
        pll.register_clkin(platform.request("clk50"), 50e6)
        if cpu_clk_freq is not None:
            # A SoC CPU takes its clock from the PLL at PLL_R[0]; driving every
            # output from that one PLL keeps the placement constraint
            # unambiguous, and its VCO reaches the CPU frequency.
            self.cd_cpu = ClockDomain()
            pll.vco_freq_range = (650e6, 1300e6)
            pll.create_clkout(self.cd_cpu, cpu_clk_freq, margin=0, with_reset=False)
            platform.toolchain.additional_cst_commands.append('INS_LOC "PLL" PLL_R[0]')
        sys_clkout = len(pll.clkouts)
        pll.create_clkout(self.cd_sys,     62.5e6, margin=0)
        # The 8-bit WR core divides the 125 MHz inputs by two for DDMTD. The
        # helper offset is applied as dynamic phase steps of this output.
        dmtd_clkout = len(pll.clkouts)
        pll.create_clkout(self.cd_wr_dmtd, 62.5e6, margin=0)
        pll.expose_dpa(clkout=dmtd_clkout)
        # Gowin removes the clock-domain aliases; the constraints name these nets.
        self.sys_clk  = pll.clkouts[sys_clkout].clk
        self.dmtd_clk = pll.clkouts[dmtd_clkout].clk

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCMini):
    def add_csr_bridge(self, *args, **kwargs):
        if self.wr_cpu_type == "ae350":
            # LiteX registers the CSR bridge only for SoCs with SDRAM. The
            # shared read path, from the bus arbiter's grant through every
            # bank, does not meet 62.5 MHz with the hard CPU without it.
            kwargs["with_register"] = True
        return super().add_csr_bridge(*args, **kwargs)

    def __init__(self, sfp=0,
        wr_cpu_type    = "urv",
        wr_cpu_variant = None,
        with_analyzer  = False,
        analyzer_csv   = "analyzer.csv",
        cpu_firmware   = None,
        main_center    = 1 << 20,
        main_shift     = 1,
    ):
        if wr_cpu_type not in WR_CPU_CORE_TYPES:
            raise ValueError(f"Unsupported WR CPU: {wr_cpu_type}")
        if wr_cpu_variant is not None and wr_cpu_type != "vexriscv":
            raise ValueError(f"The {wr_cpu_type} WR CPU does not accept a variant.")
        self.wr_cpu_type = wr_cpu_type
        wr_cpu_variant   = resolve_wr_cpu_variant(WR_CPU_CORE_TYPES[wr_cpu_type], wr_cpu_variant)
        # Each CPU runs its own firmware profile; the core's private memory
        # takes the .bram image and a SoC memory the .bin one.
        cpu_firmware = cpu_firmware or str(FIRMWARE_DIR /
            wr_cpu_firmware_filename(wr_cpu_type, "bram", "tang_mega_138k_pro"))
        cpu_binary   = str(Path(cpu_firmware).with_suffix(".bin"))

        platform = sipeed_tang_mega_138k_pro.Platform()
        platform.add_extension(_dock_i2c_io)
        platform.add_extension(_pmod0_pps_io)
        # Replicate high-fanout reset/control nets in the 125 MHz WR logic and
        # use the highest placement/routing efforts: the WR endpoint's receive
        # PCS reset, its packet filter and the uRV paths on the system clock
        # otherwise close only intermittently.
        platform.toolchain.options["maxfan"]       = 16
        platform.toolchain.options["place_option"] = 3
        platform.toolchain.options["route_option"] = 2
        self.crg = CRG(platform, cpu_clk_freq=AE350_CLK_FREQ if wr_cpu_type == "ae350" else None)

        ident = "LiteX WR on Tang Mega 138K Pro"
        if wr_cpu_type != "urv":
            ident += f" ({WR_CPU_NAMES[wr_cpu_type]})"
        soc_kwargs = {}
        if wr_cpu_type == "ae350":
            # The device's hard CPU runs WRPC instead of the LiteX BIOS. Its
            # reset address is fixed, so the ROM there holds a stub that jumps
            # to the image, which sits in fabric memory at WRPC's own linked
            # address.
            soc_kwargs = dict(
                cpu_type             = "gowin_ae350",
                integrated_rom_size  = 0x1000,
                integrated_rom_init  = AE350_BOOT_STUB,
                integrated_sram_size = FIRMWARE_SIZE,
                integrated_sram_init = get_mem_data(cpu_binary,
                    endianness = "little",
                    mem_size   = FIRMWARE_SIZE,
                ),
            )
        SoCMini.__init__(self, platform, 62.5e6, ident=ident, **soc_kwargs)
        if wr_cpu_type == "ae350":
            assert self.bus.regions["sram"].origin == AE350_FIRMWARE_ORIGIN

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
        cpu_kwargs = {}
        if wr_cpu_type == "urv":
            # The uRV runs from a single-cycle LiteX RAM inside the core; the
            # host reaches it at wr_cpu_ram for firmware loading and debug.
            cpu_kwargs = dict(
                cpu_memory_region = SoCRegion(origin=WR_CPU_LOCAL_MEMORY_ORIGIN, size=FIRMWARE_SIZE),
                cpu_memory_local  = True,
            )
        elif wr_cpu_type == "vexriscv":
            # A LiteX CPU is outside the WR core and runs from SoC memory.
            self.add_ram("wr_cpu_mem", WR_CPU_MEMORY_ORIGIN, WR_CPU_MEMORY_SIZE,
                contents=get_mem_data(cpu_binary,
                    data_width = 32,
                    endianness = "little",
                    mem_size   = WR_CPU_MEMORY_SIZE,
                ))
            cpu_kwargs = dict(
                cpu_variant       = wr_cpu_variant,
                cpu_memory_region = SoCRegion(origin=WR_CPU_MEMORY_ORIGIN,
                    size=WR_CPU_MEMORY_SIZE, mode="rwx"),
            )
        wr = add_white_rabbit(self,
            cpu_firmware      = cpu_firmware,
            cpu_type          = WR_CPU_CORE_TYPES[wr_cpu_type],
            board_name        = "T138",
            phy               = self.phy,
            with_ext_clk      = False,
            serial_pads       = wr_serial,
            sfp_los_pads      = pads.los,
            sfp_disable_pads  = pads.tx_disable,
            **cpu_kwargs,
        )
        self.comb += [
            wr.source.ready.eq(1),
            wr.sink.valid.eq(0),
        ]
        if wr_cpu_type == "ae350":
            # WRPC's peripherals, at the address the firmware was built for,
            # in the CPU's uncached peripheral range.
            self.bus.add_slave(name="wr_cpu_periph", slave=wr.cpu_peripheral_bus,
                region=SoCRegion(origin=AE350_PERIPHERAL_ORIGIN, size=AE350_PERIPHERAL_SIZE,
                    cached=False))
            # The SoftPLL interrupt drives the CPU's first user interrupt
            # input; the firmware enables that PLIC source and completes every
            # claim.
            assert not self.irq.locs, "GP_INT[0] is reserved for the WR SoftPLL."
            self.comb += self.cpu.interrupt[0].eq(wr.cpu_irq)
            self.wr_cpu_status = CSRStatus(fields=[
                CSRField("irq",   description="WRPC SoftPLL interrupt request."),
                CSRField("reset", description="WRPC's reset request for the SoC's CPU."),
            ])
            self.comb += [
                self.wr_cpu_status.fields.irq.eq(wr.cpu_irq),
                self.wr_cpu_status.fields.reset.eq(wr.cpu_reset),
            ]

        # Clock actuators --------------------------------------------------------------------------
        # Helper: DDMTD offset from dynamic phase steps of the 62.5 MHz PLL output.
        self.helper_tuning = helper_tuning = WRGowinPLLBackend(cd="sys")
        self.comb += [
            helper_tuning.command.load.eq(wr.dac_dmtd_load),
            helper_tuning.command.data.eq(wr.dac_dmtd_data),
            self.crg.pll.phase_dir.eq(helper_tuning.phase_dir),
            self.crg.pll.phase_step.eq(helper_tuning.phase_step),
        ]
        # Main: fractional feedback of the dock MS5351 generating the 100 MHz
        # SerDes reference (PLL0, 900 MHz VCO from 25 MHz, documented setup).
        # Shift 1 spans +/- 14 ppm around the center in 0.4 ppb steps; larger
        # shifts did not phase-lock with the I2C update latency.
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

        # SFP EEPROM -------------------------------------------------------------------------------
        # The module EEPROM shares the dock I2C bus with the clock generator,
        # and WRPC bit-bangs it whenever it likes. Copy it into a memory while
        # the reader owns the bus, and serve that copy to the firmware.
        self.sfp_eeprom = sfp_eeprom = SFPEEPROMCache(62.5e6)
        self.comb += [
            sfp_eeprom.emulator.scl_i.eq(wr.sfp_scl_o),
            sfp_eeprom.emulator.sda_i.eq(wr.sfp_sda_o),
            wr.sfp_scl_i.eq(wr.sfp_scl_o),
            wr.sfp_sda_i.eq(wr.sfp_sda_o & sfp_eeprom.emulator.sda_o),
        ]

        # Dock I2C ---------------------------------------------------------------------------------
        # One owner at a time: the clock tuner, then the EEPROM reader between
        # its updates, then the host bit-bang master. Each owner also selects
        # its multiplexer channel.
        self.i2c     = I2CMaster(connect_pads=False)
        self.i2c_sel = CSRStorage(3, reset=DOCK_I2C_CHANNEL_MS5351,
            description="Dock I2C multiplexer channel for host accesses.")
        i2c_pads  = platform.request("dock_i2c")
        tuner     = main_tuning.tuner
        reader    = sfp_eeprom.reader
        scl_i     = Signal()
        sda_i     = Signal()
        # The tuner is idle between updates; grant the reader one of those gaps
        # and hold the grant for the whole transfer.
        self.comb += reader.grant.eq(tuner.enable & ~tuner.busy)
        scl_o = Signal(reset=1)
        sda_o = Signal(reset=1)
        self.comb += [
            If(reader.busy,
                scl_o.eq(reader.scl_o),
                sda_o.eq(reader.sda_o),
            ).Elif(tuner.enable,
                scl_o.eq(tuner.scl_o),
                sda_o.eq(tuner.sda_o),
            ).Else(
                scl_o.eq(self.i2c._w.fields.scl),
                sda_o.eq(~(self.i2c._w.fields.oe & ~self.i2c._w.fields.sda)),
            ),
        ]
        self.specials += [
            # I2C uses pull-ups: only drive low.
            Tristate(i2c_pads.scl, o=0, i=scl_i, oe=~scl_o),
            Tristate(i2c_pads.sda, o=0, i=sda_i, oe=~sda_o),
        ]
        self.comb += [
            tuner.sda_i.eq(sda_i),
            reader.sda_i.eq(sda_i),
            self.i2c._r.fields.scl.eq(scl_i),
            self.i2c._r.fields.sda.eq(sda_i),
            platform.request("dock_i2c_sel").eq(
                Mux(reader.busy, DOCK_I2C_CHANNEL_SFP + sfp,
                Mux(tuner.enable, DOCK_I2C_CHANNEL_MS5351, self.i2c_sel.storage))),
        ]

        # PPS --------------------------------------------------------------------------------------
        # Output the WR PPS with a 20% duty cycle, as on SPEC-A7, and timestamp
        # an external PPS with eight samples per reference cycle plus IODELAY
        # taps for sub-nanosecond scans.
        pps_out = Signal()
        self.pps_out_gen = PPSGenerator(i=wr.pps_out_pulse, o=pps_out, clk_domain="wr", clk_freq=125e6)
        self.comb += platform.request("pps_out").eq(pps_out)
        # Timestamp the generated pulse against the same WR time: its edge
        # qualifies the output path without an external connection, and lands
        # at the start of a WR second when the servo is locked.
        pps_out_sampled = Signal()
        self.sync.wr += pps_out_sampled.eq(pps_out)
        self.pps_out_timestamper = PPSTimestamper(pps_out_sampled,
            tm_seconds = wr.tm_seconds,
            tm_cycles  = wr.tm_cycles,
            tm_valid   = wr.tm_time_valid,
            cd         = "wr",
        )
        self.pps_sampler = pps_sampler = GW5OversampledInput(platform,
            pad = platform.request("pps_in"),
            clk = ClockSignal("wr"),
        )
        pps_samples = Signal(8)
        self.sync.wr += pps_samples.eq(pps_sampler.samples)
        self.pps_timestamper = PPSTimestamper(pps_samples,
            tm_seconds = wr.tm_seconds,
            tm_cycles  = wr.tm_cycles,
            tm_valid   = wr.tm_time_valid,
            cd         = "wr",
        )

        # Diagnostics ------------------------------------------------------------------------------
        self.ref_clk_freq  = FreqMeter(62_500_000, clk=ClockSignal("wr"))
        self.dmtd_clk_freq = FreqMeter(62_500_000, clk=ClockSignal("wr_dmtd"))
        self.rx_clk_freq   = FreqMeter(62_500_000, clk=self.phy.rx_clk)
        if with_analyzer:
            from litescope import LiteScopeAnalyzer
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
        sys_clk  = self.crg.sys_clk
        dmtd_clk = self.crg.dmtd_clk
        platform.add_generated_clock_constraint(sys_clk, platform.lookup_request("clk50"),
            divide_by=4, multiply_by=5, name="sys")
        platform.add_generated_clock_constraint(dmtd_clk, platform.lookup_request("clk50"),
            divide_by=4, multiply_by=5, name="dmtd")
        platform.add_period_constraint(self.phy.tx_clk, 8)
        platform.add_period_constraint(self.phy.rx_clk, 8)
        # The PPS sampler clocks are generated from the WR reference; the word
        # clock's paths to the reference domain are timed, not excluded. The
        # IODELAY taps (sys) and the deserializer's asynchronous reset (word)
        # are quasi-static controls of the sampling clock domain.
        pps_fast_clk = pps_sampler.pll.clkouts[0].clk
        pps_word_clk = pps_sampler.pll.clkouts[1].clk
        platform.add_generated_clock_constraint(pps_fast_clk, self.phy.tx_clk,
            divide_by=1, multiply_by=4, name="pps_fast")
        platform.add_generated_clock_constraint(pps_word_clk, self.phy.tx_clk,
            divide_by=1, multiply_by=1, name="pps_word")
        platform.add_false_path_constraints(
            sys_clk, dmtd_clk, self.phy.tx_clk, self.phy.rx_clk)
        platform.add_false_path_constraints(sys_clk, pps_word_clk, pps_fast_clk)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="White Rabbit on Tang Mega 138K Pro.")

    # Build/load options.
    parser.add_argument("--build",               action="store_true", help="Build the bitstream.")
    parser.add_argument("--load",                action="store_true", help="Load the bitstream.")
    parser.add_argument("--skip-firmware-build", action="store_true", help="Reuse the existing WR firmware.")
    parser.add_argument("--output-dir", default=None, help="Build output directory.")

    # WR CPU options.
    parser.add_argument("--wr-cpu-type", default="urv", choices=WR_CPU_PROFILES,
        help="CPU running the WR firmware (default: embedded uRV).")
    parser.add_argument("--wr-cpu-variant", default=None,
        help="LiteX WR CPU variant (VexRiscv defaults to lite).")

    # PHY/debug options.
    parser.add_argument("--sfp", type=int, choices=(0, 1), default=0, help="SFP lane (default: 0).")
    parser.add_argument("--main-center", type=lambda v: int(v, 0), default=1 << 20,
        help="MS5351 code for a neutral main-clock command (default: 0x100000, nominal).")
    parser.add_argument("--main-shift", type=int, default=1,
        help="MS5351 codes per main-clock command step, as a power of two (default: 1).")
    parser.add_argument("--with-analyzer", action="store_true",
        help="Capture raw and decoded RX symbols with LiteScope.")
    args = parser.parse_args()
    if args.output_dir is None:
        suffix = "" if args.wr_cpu_type == "urv" else f"_{args.wr_cpu_type}"
        args.output_dir = f"build/tang_mega_138k_pro_wr{suffix}"

    # Firmware: each CPU has its own profile, and a SoC CPU also needs the
    # address its SoC decodes WRPC's peripheral window at.
    if not args.skip_firmware_build:
        command = [
            sys.executable, str(FIRMWARE_DIR / "build.py"),
            "--target", "tang_mega_138k_pro", "--read-only-storage",
            "--wr-cpu-type", args.wr_cpu_type,
        ]
        if args.wr_cpu_type == "ae350":
            command += ["--peripheral-origin", hex(AE350_PERIPHERAL_ORIGIN)]
        subprocess.run(command, check=True)

    # Gateware.
    soc = BaseSoC(
        sfp            = args.sfp,
        wr_cpu_type    = args.wr_cpu_type,
        wr_cpu_variant = args.wr_cpu_variant,
        with_analyzer  = args.with_analyzer,
        analyzer_csv   = str(Path(args.output_dir) / "analyzer.csv"),
        main_center    = args.main_center,
        main_shift     = args.main_shift,
    )
    builder = Builder(soc,
        output_dir = args.output_dir,
        csr_csv    = str(Path(args.output_dir) / "csr.csv"),
        # A SoC CPU runs WRPC, not the LiteX BIOS.
        compile_software = args.wr_cpu_type != "ae350",
    )
    builder.build(run=args.build)
    if args.load:
        soc.platform.create_programmer().load_bitstream(builder.get_bitstream_filename(mode="sram"))


if __name__ == "__main__":
    main()
