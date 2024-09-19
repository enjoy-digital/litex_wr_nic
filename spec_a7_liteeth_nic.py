#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.build.io             import DifferentialInput
from litex.build.openfpgaloader import OpenFPGALoader

from spec_a7_platform import *

from litex.soc.interconnect.csr     import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.clock import *
from litex.soc.cores.led   import LedChaser

from litepcie.software      import generate_litepcie_software_headers
from litepcie.phy.s7pciephy import S7PCIEPHY

from liteeth.phy.a7_gtp       import QPLLSettings, QPLL
from liteeth.phy.a7_1000basex import A7_1000BASEX

from litepcie.software import generate_litepcie_software

from gateware.qpll         import SharedQPLL
from gateware.nic.pcie_nic import PCIeNICSoC

# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_eth=False):
        self.rst      = Signal()
        self.cd_sys   = ClockDomain()
        self.cd_refclk_pcie = ClockDomain()
        self.cd_refclk_eth  = ClockDomain()

        # # #

        # Clk/Rst.
        clk62p5 = platform.request("clk62p5")

        # PLL.
        self. pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk62p5, 62.5e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, margin=0)
        if with_eth:
            self.cd_eth_ref = ClockDomain()
            pll.create_clkout(self.cd_eth_ref, 156.25e6, margin=0)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(PCIeNICSoC):
    def __init__(self, sys_clk_freq=125e6, with_led_chaser=True, **kwargs):
        # Platform ---------------------------------------------------------------------------------

        platform = Platform(variant="xc7a50t")

        # Clocking ---------------------------------------------------------------------------------

        self.crg = CRG(platform, sys_clk_freq, with_eth=True)

        # Shared QPLL.
        self.qpll = SharedQPLL(platform,
            with_pcie       = True,
            with_eth        = True,
            eth_refclk_freq = 156.25e6,
        )

        # SoCCore ----------------------------------------------------------------------------------

        SoCMini.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on SPEC-A7", ident_version=True)

        # JTAGBone ---------------------------------------------------------------------------------

        self.add_jtagbone()

        # PCIe -------------------------------------------------------------------------------------

        self.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1"),
            data_width = 64,
            bar0_size  = 0x20000,
        )
        self.comb += ClockSignal("refclk_pcie").eq(self.pcie_phy.pcie_refclk)
        self.pcie_phy.use_external_qpll(qpll_channel=self.qpll.get_channel("pcie"))
        platform.toolchain.pre_placement_commands.append("reset_property LOC [get_cells -hierarchical -filter {{NAME=~pcie_s7/*gtp_channel.gtpe2_channel_i}}]")
        platform.toolchain.pre_placement_commands.append("set_property LOC GTPE2_CHANNEL_X0Y0 [get_cells -hierarchical -filter {{NAME=~pcie_s7/*gtp_channel.gtpe2_channel_i}}]")

        # Ethernet ---------------------------------------------------------------------------------

        self.comb += platform.request("sfp_disable").eq(0)
        self.ethphy = A7_1000BASEX(
            qpll_channel = self.qpll.get_channel("eth"),
            data_pads    = self.platform.request("sfp"),
            sys_clk_freq = sys_clk_freq,
            rx_polarity  = 0,
            tx_polarity  = 0,
        )

        # PCIe NIC ---------------------------------------------------------------------------------

        self.add_pcie_nic(pcie_phy=self.pcie_phy, eth_phy=self.ethphy)

        # Leds -------------------------------------------------------------------------------------

        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request("user_led", 0),
                sys_clk_freq = sys_clk_freq)
        self.comb += platform.request("user_led", 2).eq(~self.ethphy.link_up) # Inverted.

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX-LiteEth-NIC on SPEC-A7.")

    # Build/Load/Flash Arguments.
    # ---------------------------
    parser.add_argument("--build", action="store_true", help="Build bitstream.")
    parser.add_argument("--load",  action="store_true", help="Load bitstream.")
    parser.add_argument("--flash", action="store_true", help="Flash bitstream.")

    args = parser.parse_args()

    # Build SoC.
    # ----------
    soc = BaseSoC()
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build)

    # Generate PCIe C Headers.
    # ------------------------
    generate_litepcie_software_headers(soc, "software/driver")

    # Generate Bitstream.
    # -------------------
    if args.load or args.flash:
        os.system("python3 gateware/xilinx-bitstream.py {bit_file} {bin_file}".format(
            bit_file = builder.get_bitstream_filename(mode="sram"),
            bin_file = builder.get_bitstream_filename(mode="flash"),
        ))

    # Load FPGA.
    # ----------
    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    # Flash FPGA.
    # -----------
    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, builder.get_bitstream_filename(mode="flash"))

if __name__ == "__main__":
    main()
