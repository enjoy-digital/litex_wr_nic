#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2021-2024 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.build.generic_platform import Subsignal, Pins
from litex.build.io import DifferentialInput
from litex.build.openfpgaloader import OpenFPGALoader

from litex_boards.platforms import sqrl_acorn

from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litex.soc.cores.clock import *
from litex.soc.cores.led import LedChaser

from litex.build.generic_platform import IOStandard, Subsignal, Pins

from litepcie.phy.s7pciephy import S7PCIEPHY

from liteeth.phy.a7_gtp import QPLLSettings, QPLL
from liteeth.phy.a7_1000basex import A7_1000BASEX

from litepcie.software import generate_litepcie_software

from gateware.eth_pcie_soc import EthernetPCIeSoC

# Platform -----------------------------------------------------------------------------------------

class Platform(sqrl_acorn.Platform):
    def create_programmer(self):
        return OpenFPGALoader(cable="ft2232", fpga_part=f"xc7a200tfbg484", freq=10e6)

_serial_io = [
    ("serial", 0,
        Subsignal("tx", Pins("G1"),  IOStandard("LVCMOS33")), # CLK_REQ
        Subsignal("rx", Pins("Y13"), IOStandard("LVCMOS18")), # SMB_ALERT_N
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_eth=False):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()

        # Clk/Rst.
        clk200    = platform.request("clk200")
        clk200_se = Signal()
        self.specials += DifferentialInput(clk200.p, clk200.n, clk200_se)

        # PLL.
        self.pll = pll = S7PLL()
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200_se, 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

        # Eth PLL.
        if with_eth:
            self.cd_eth_ref = ClockDomain()
            self.eth_pll = eth_pll = S7PLL()
            self.comb += eth_pll.reset.eq(self.rst)
            eth_pll.register_clkin(clk200_se, 200e6)
            eth_pll.create_clkout(self.cd_eth_ref, 156.25e6, margin=0)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(EthernetPCIeSoC):
    def __init__(self, sys_clk_freq=125e6,
        with_led_chaser = True,
        **kwargs):
        platform = Platform(variant="cle-215+")
        platform.add_extension(_serial_io, prepend=True)

        # CRG --------------------------------------------------------------------------------------
        self.crg = CRG(platform, sys_clk_freq, with_eth=True)

        # SoCCore ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on Acorn CLE-101/215(+)")
        self.add_jtagbone()

        # PCIe / Ethernet Shared QPLL Settings -----------------------------------------------------

        # PCIe QPLL Settings.
        qpll_pcie_settings = QPLLSettings(
            refclksel  = 0b001,
            fbdiv      = 5,
            fbdiv_45   = 5,
            refclk_div = 1,
        )

        # Ethernet QPLL Settings.
        qpll_eth_settings = QPLLSettings(
            refclksel  = 0b111,
            fbdiv      = 4,
            fbdiv_45   = 4,
            refclk_div = 1,
        )
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

        # PCIe -------------------------------------------------------------------------------------

        self.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1_baseboard"),
            pcie_data_width = 64,
            data_width      = 128,
            bar0_size       = 0x20000,
        )
        platform.toolchain.pre_placement_commands.append("reset_property LOC [get_cells -hierarchical -filter {{NAME=~pcie_s7/*gtp_channel.gtpe2_channel_i}}]")
        platform.toolchain.pre_placement_commands.append("set_property LOC GTPE2_CHANNEL_X0Y7 [get_cells -hierarchical -filter {{NAME=~pcie_s7/*gtp_channel.gtpe2_channel_i}}]")
        #self.add_pcie(phy=self.pcie_phy, ndmas=1)

        # Shared QPLL ------------------------------------------------------------------------------
        self.qpll = qpll = QPLL(
            gtrefclk0     = self.pcie_phy.pcie_refclk,
            qpllsettings0 = qpll_pcie_settings,
            gtgrefclk1    = self.crg.cd_eth_ref.clk,
            qpllsettings1 = qpll_eth_settings,
        )
        self.pcie_phy.use_external_qpll(qpll_channel=qpll.channels[0])

        # Ethernet ---------------------------------------------------------------------------------
        _eth_io = [
            ("sfp", 0,
                Subsignal("txp", Pins(" D5")),
                Subsignal("txn", Pins(" C5")),
                Subsignal("rxp", Pins("D11")),
                Subsignal("rxn", Pins("C11")),
            ),
        ]
        platform.add_extension(_eth_io)

        self.ethphy = A7_1000BASEX(
            qpll_channel = qpll.channels[1],
            data_pads    = self.platform.request("sfp"),
            sys_clk_freq = sys_clk_freq,
            rx_polarity  = 1,  # Inverted on Acorn.
            tx_polarity  = 0   # Inverted on Acorn and on baseboard.
        )
        #self.add_etherbone(phy=self.ethphy, ip_address="192.168.1.50")

        # PCIe + Ethernet --------------------------------------------------------------------------
        self.add_ethernet_pcie(phy=self.ethphy, pcie_phy=self.pcie_phy)

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request("user_led", 0),
                sys_clk_freq = sys_clk_freq)
        self.comb += platform.request("user_led", 2).eq(~self.ethphy.link_up) # Inverted.

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sqrl_acorn.Platform, description="LiteX SoC on Acorn CLE-101/215(+).")
    parser.add_target_argument("--sys-clk-freq", default=125e6, type=float, help="System clock frequency.")
    parser.add_argument("--flash", action="store_true", help="Flash bitstream.")
    args = parser.parse_args()

    soc = BaseSoC(
        sys_clk_freq = args.sys_clk_freq,
        **parser.soc_argdict
    )
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)
        soc.generate_software_header("driver")

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, builder.get_bitstream_filename(mode="flash"))

if __name__ == "__main__":
    main()
