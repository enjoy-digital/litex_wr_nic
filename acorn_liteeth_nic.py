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

from litex_boards.platforms import sqrl_acorn

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

from gateware.qpll import SharedQPLL
from gateware.soc  import LiteXWRNICSoC

# Platform -----------------------------------------------------------------------------------------

class Platform(sqrl_acorn.Platform):
    def detect_ftdi_chip(self):
        lsusb_log = subprocess.run(['lsusb'], capture_output=True, text=True)
        for ftdi_chip in ["ft232", "ft2232", "ft4232"]:
            if f"Future Technology Devices International, Ltd {ftdi_chip.upper()}" in lsusb_log.stdout:
                return ftdi_chip
        return None

    def create_programmer(self):
        ftdi_chip = self.detect_ftdi_chip()
        if ftdi_chip is None:
            raise RuntimeError("No compatible FTDI device found.")
        return OpenFPGALoader(cable=ftdi_chip, fpga_part="xc7a200tfbg484", freq=10e6)

# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_eth=False):
        self.rst      = Signal()
        self.cd_sys   = ClockDomain()
        self.cd_refclk_pcie = ClockDomain()
        self.cd_refclk_eth  = ClockDomain()

        # # #

        # Clk/Rst.
        clk200    = platform.request("clk200")
        clk200_se = Signal()
        self.specials += DifferentialInput(clk200.p, clk200.n, clk200_se)

        # PLL.
        self.pll = pll = S7PLL()
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200_se, 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin)

        # Eth PLL.
        if with_eth:
            self.eth_pll = eth_pll = S7PLL()
            self.comb += eth_pll.reset.eq(self.rst)
            eth_pll.register_clkin(clk200_se, 200e6)
            eth_pll.create_clkout(self.cd_refclk_eth, 156.25e6, margin=0)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(LiteXWRNICSoC):
    def __init__(self, sys_clk_freq=125e6, with_led_chaser=True, **kwargs):
        # Platform ---------------------------------------------------------------------------------

        platform = Platform(variant="cle-215+")
        platform.add_extension(sqrl_acorn._litex_acorn_baseboard_mini_io, prepend=True)

        # Clocking ---------------------------------------------------------------------------------

        self.crg = CRG(platform, sys_clk_freq, with_eth=True)

        # Shared QPLL.
        self.qpll = SharedQPLL(platform,
            with_pcie           = True,
            with_eth            = True,
            eth_refclk_freq     = 156.25e6,
            eth_refclk_from_pll = True,
        )
        self.qpll.enable_pll_refclk()

        # SoCMini ----------------------------------------------------------------------------------

        SoCMini.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on Acorn CLE-101/215(+)", ident_version=True)

        # JTAGBone ---------------------------------------------------------------------------------

        self.add_jtagbone()
        platform.add_period_constraint(self.jtagbone_phy.cd_jtag.clk, 1e9/20e6)
        platform.add_false_path_constraints(self.jtagbone_phy.cd_jtag.clk, self.crg.cd_sys.clk)

        # PCIe -------------------------------------------------------------------------------------

        self.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1"),
            data_width = 64,
            bar0_size  = 0x20000,
        )
        self.pcie_phy.update_config({
            "Base_Class_Menu"          : "Network_controller",
            "Sub_Class_Interface_Menu" : "Ethernet_controller",
            "Class_Code_Base"          : "02",
            "Class_Code_Sub"           : "00",
        })
        self.comb += ClockSignal("refclk_pcie").eq(self.pcie_phy.pcie_refclk)
        self.pcie_phy.use_external_qpll(qpll_channel=self.qpll.get_channel("pcie"))
        platform.toolchain.pre_placement_commands.append("reset_property LOC [get_cells -hierarchical -filter {{NAME=~pcie_s7/*gtp_channel.gtpe2_channel_i}}]")
        platform.toolchain.pre_placement_commands.append("set_property LOC GTPE2_CHANNEL_X0Y7 [get_cells -hierarchical -filter {{NAME=~pcie_s7/*gtp_channel.gtpe2_channel_i}}]")

        # Ethernet ---------------------------------------------------------------------------------

        self.ethphy = A7_1000BASEX(
            qpll_channel = self.qpll.get_channel("eth"),
            data_pads    = self.platform.request("sfp"),
            sys_clk_freq = sys_clk_freq,
            rx_polarity  = 1,  # Inverted on Acorn.
            tx_polarity  = 0,  # Inverted on Acorn and on baseboard.
        )
        self.platform.add_period_constraint(self.ethphy.txoutclk, 1e9/62.5e6)
        self.platform.add_period_constraint(self.ethphy.rxoutclk, 1e9/62.5e6)
        platform.add_false_path_constraints(self.ethphy.txoutclk, self.ethphy.rxoutclk, self.crg.cd_sys.clk)

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
    parser = argparse.ArgumentParser(description="LiteX-LiteEth-NIC on Acorn Baseboard Mini.")

    # Build/Load/Flash Arguments.
    # ---------------------------
    parser.add_argument("--build", action="store_true", help="Build bitstream.")
    parser.add_argument("--load",  action="store_true", help="Load bitstream.")
    parser.add_argument("--flash", action="store_true", help="Flash bitstream.")

    args = parser.parse_args()

    # Build SoC.
    # ----------
    soc = BaseSoC()
    builder = Builder(soc, csr_csv="test/csr.csv")
    builder.build(run=args.build)

    # Generate PCIe C Headers.
    # ------------------------
    generate_litepcie_software_headers(soc, "software/kernel")

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
