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

from liteeth.phy.a7_gtp import QPLLSettings, QPLL
from liteeth.phy.a7_1000basex import A7_1000BASEX

from litex.soc.cores.bitbang import I2CMaster

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_eth=False):
        self.rst      = Signal()
        self.cd_sys   = ClockDomain()
        self.cd_sys2x = ClockDomain()

        # # #

        # Clk/Rst.
        clk125m_oe = platform.request("clk125m_oe")
        clk125m    = platform.request("clk125m")
        self.comb += clk125m_oe.eq(1)

        # PLL.
        self. pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk125m, 125e6)
        pll.create_clkout(self.cd_sys,       sys_clk_freq, margin=0)
        pll.create_clkout(self.cd_sys2x, 2 * sys_clk_freq, margin=0)
        if with_eth:
            self.cd_eth_ref = ClockDomain()
            pll.create_clkout(self.cd_eth_ref, 156.25e6, margin=0)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=125e6,  with_hyperram=False, with_ethernet=False, with_etherbone=False, eth_sfp=0, **kwargs):
        platform = Platform(variant="xc7a35t")

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq, with_eth=with_ethernet or with_etherbone)

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

        # Ethernet / Etherbone ---------------------------------------------------------------------
        if with_ethernet or with_etherbone:
            # Ethernet QPLL Settings.
            qpll_eth_settings = QPLLSettings(
                refclksel  = 0b111,
                fbdiv      = 4,
                fbdiv_45   = 4,
                refclk_div = 1,
            )
            platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")
            # Shared QPLL.
            self.qpll = qpll = QPLL(
                gtrefclk0     = self.crg.cd_eth_ref.clk,
                qpllsettings0 = qpll_eth_settings,
            )
            self.comb += platform.request("sfp_disable", eth_sfp).eq(0)
            self.ethphy = A7_1000BASEX(
                qpll_channel = qpll.channels[0],
                data_pads    = self.platform.request("sfp", eth_sfp),
                sys_clk_freq = sys_clk_freq,
                rx_polarity  = 0,
                tx_polarity  = 0,
            )
            if with_etherbone:
                self.add_etherbone(phy=self.ethphy, ip_address="192.168.1.50")
            elif with_ethernet:
                self.add_ethernet(phy=self.ethphy, local_ip="192.168.1.50", remote_ip="192.168.1.100")

        # Frontpanel Leds --------------------------------------------------------------------------
        self.leds = LedChaser(
            pads         = platform.request_all("frontpanel_led"),
            sys_clk_freq = sys_clk_freq,
        )

        # SFP I2C ----------------------------------------------------------------------------------

        i2c_pads = platform.request("sfp_i2c", 0)
        self.i2c = I2CMaster(pads=i2c_pads)

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=Platform, description="LiteX SoC on SPEC-A7")
    parser.add_target_argument("--sys-clk-freq",   default=125e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--with-hyperram",  action="store_true",       help="Add HyperRAM.")
    parser.add_target_argument("--with-ethernet",  action="store_true",       help="Enable Ethernet support.")
    parser.add_target_argument("--with-etherbone", action="store_true",       help="Enable Etherbone support.")
    parser.add_target_argument("--eth-sfp",        default=0, type=int,       help="Ethernet/Etherbone SFP Connector", choices=[0, 1])
    args = parser.parse_args()

    soc = BaseSoC(
        sys_clk_freq   = args.sys_clk_freq,
        with_hyperram  = args.with_hyperram,
        with_ethernet  = args.with_ethernet,
        with_etherbone = args.with_etherbone,
        eth_sfp        = args.eth_sfp,
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
