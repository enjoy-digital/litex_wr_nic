#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from spec_a7_platform import Platform

from litex.soc.interconnect import wishbone

from litex.soc.integration.soc_core import *
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.builder  import *

from litex.soc.cores.clock    import *
from litex.soc.cores.led      import LedChaser
from litex.soc.cores.hyperbus import HyperRAM

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst      = Signal()
        self.cd_sys   = ClockDomain()
        self.cd_sys2x = ClockDomain()

        # # #

        # Clk/Rst.
        clk62p5 = platform.request("clk62p5")

        # PLL.
        self. pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk62p5, 62.5e6)
        pll.create_clkout(self.cd_sys,       sys_clk_freq, margin=0)
        pll.create_clkout(self.cd_sys2x, 2 * sys_clk_freq, margin=0)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=100e6,  with_hyperram=False, **kwargs):
        platform = Platform()

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on SPEC-A7", **kwargs)

        # HyperRAM ---------------------------------------------------------------------------------
        if with_hyperram:
            # HyperRAM Parameters.
            hyperram_device     = "S27KS0642"
            hyperram_size       = 8  * MEGABYTE
            hyperram_cache_size = 0  * KILOBYTE

            # HyperRAM Bus/Slave Interface.
            hyperram_bus = wishbone.Interface(data_width=32, address_width=32, addressing="word")
            self.bus.add_slave(name="main_ram", slave=hyperram_bus, region=SoCRegion(origin=0x40000000, size=hyperram_size, mode="rwx"))

            # HyperRAM L2 Cache.
            hyperram_cache = wishbone.Cache(
                cachesize = hyperram_cache_size//4,
                master    = hyperram_bus,
                slave     = wishbone.Interface(data_width=32, address_width=32, addressing="word")
            )
            hyperram_cache = FullMemoryWE()(hyperram_cache)
            self.hyperram_cache = hyperram_cache
            self.add_config("L2_SIZE", hyperram_cache_size)

            # HyperRAM Core.
            self.hyperram = HyperRAM(
                pads         = platform.request("hyperram"),
                latency      = 7,
                latency_mode = "variable",
                sys_clk_freq = sys_clk_freq,
                clk_ratio    = "2:1",
            )
            self.comb += self.hyperram_cache.slave.connect(self.hyperram.bus)

        # Frontpanel Leds --------------------------------------------------------------------------
        self.leds = LedChaser(
            pads         = platform.request_all("frontpanel_led"),
            sys_clk_freq = sys_clk_freq,
        )

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=Platform, description="LiteX SoC on SPEC-A7")
    parser.add_target_argument("--sys-clk-freq",  default=100e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--with-hyperram", action="store_true",       help="Add HyperRAM.")
    args = parser.parse_args()

    soc = BaseSoC(
        sys_clk_freq  = args.sys_clk_freq,
        with_hyperram = args.with_hyperram,
        **parser.soc_argdict
    )
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
