#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse

from migen.genlib.cdc import MultiReg

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex_boards.platforms import sqrl_acorn

from litex.build.generic_platform import *
from litex.build.io               import DifferentialInput, DifferentialOutput
from litex.build.openfpgaloader   import OpenFPGALoader

from litex.soc.interconnect.csr     import *
from litex.soc.interconnect         import stream
from litex.soc.interconnect         import wishbone

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.clock          import S7PLL, S7MMCM
from litex.soc.cores.led            import LedChaser
from litex.soc.cores.spi.spi_master import SPIMaster

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.software      import generate_litepcie_software_headers

from litescope import LiteScopeAnalyzer

from litex_wr_nic.gateware.uart              import UARTShared
from litex_wr_nic.gateware.soc               import LiteXWRNICSoC
from litex_wr_nic.gateware.time              import TimeGenerator
from litex_wr_nic.gateware.qpll              import SharedQPLL
from litex_wr_nic.gateware.ad5683r.core      import AD5683RDAC
from litex_wr_nic.gateware.ad9516.core       import AD9516PLL, AD9516_MAIN_CONFIG, AD9516_EXT_CONFIG
from litex_wr_nic.gateware.measurement       import MultiClkMeasurement
from litex_wr_nic.gateware.delay.core        import MacroDelay, CoarseDelay, FineDelay
from litex_wr_nic.gateware.pps               import PPSGenerator
from litex_wr_nic.gateware.clk10m            import Clk10MGenerator
from litex_wr_nic.gateware.nic.phy           import LiteEthPHYWRGMII
from litex_wr_nic.gateware.ps_gen            import PSGen

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_white_rabbit=True):
        self.rst              = Signal()
        self.cd_sys           = ClockDomain()
        self.cd_clk200        = ClockDomain()
        self.cd_refclk_pcie   = ClockDomain()
        self.cd_refclk_eth    = ClockDomain()
        self.cd_clk_125m_gtp  = ClockDomain()
        self.cd_clk_62m5_dmtd = ClockDomain()
        self.cd_clk10m_in     = ClockDomain()
        self.cd_clk62m5_in    = ClockDomain()

        # # #

        # Sys PLL (Free-Running from clk200).
        # ----------------------------------
        clk200 = platform.request("clk200")

        self.pll = pll = S7PLL(speedgrade=-3)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200, 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, margin=0)
        self.comb += self.cd_clk200.clk.eq(pll.clkin)

        # RefClk MMCM (125MHz).
        # ---------------------
        self.refclk_mmcm = S7MMCM(speedgrade=-3)
        self.comb += self.refclk_mmcm.reset.eq(self.rst)
        self.refclk_mmcm.register_clkin(ClockSignal("clk200"), 200e6)
        self.refclk_mmcm.create_clkout(self.cd_clk_125m_gtp,  125e6, margin=0)
        self.refclk_mmcm.expose_dps("clk200", with_csr=False)
        self.refclk_mmcm.params.update(p_CLKOUT0_USE_FINE_PS="TRUE")
        self.comb += self.cd_refclk_eth.clk.eq(self.cd_clk_125m_gtp.clk)

        # DMTD MMCM (62.5MHz).
        # --------------------
        self.dmtd_mmcm = S7MMCM(speedgrade=-3)
        self.comb += self.dmtd_mmcm.reset.eq(self.rst)
        self.dmtd_mmcm.register_clkin(ClockSignal("clk200"), 200e6)
        self.dmtd_mmcm.create_clkout(self.cd_clk_62m5_dmtd, 62.5e6, margin=0)
        self.dmtd_mmcm.expose_dps("clk200", with_csr=False)
        self.dmtd_mmcm.params.update(p_CLKOUT0_USE_FINE_PS="TRUE")

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(LiteXWRNICSoC):
    def __init__(self, sys_clk_freq=125e6,
        # PCIe Parameters.
        # ----------------
        with_pcie = True,

        # White Rabbit Parameters.
        # ------------------------
        with_white_rabbit          = True,
        white_rabbit_sfp_connector = 0,
        white_rabbit_cpu_firmware  = "litex_wr_nic/firmware/spec_a7_wrc.bram",

    ):
        # Platform ---------------------------------------------------------------------------------

        platform = sqrl_acorn.Platform()
        platform.add_extension(sqrl_acorn._litex_acorn_baseboard_mini_io, prepend=True)
        platform.add_extension([
            ("pps_out",    0, Pins("H5"), IOStandard("LVCMOS33")),
            ("wr_clk_out", 0, Pins("J5"), IOStandard("LVCMOS33")),
        ])

        # Clocking ---------------------------------------------------------------------------------

        # General / WR.
        self.crg = _CRG(platform,
            sys_clk_freq      = sys_clk_freq,
            with_white_rabbit = with_white_rabbit,
        )

        # Shared QPLL.
        self.qpll = SharedQPLL(platform,
            with_pcie           = True, # Always True even when PCIe is disabled for correct WR Clocking.
            with_eth            = with_white_rabbit,
            eth_refclk_freq     = 125e6,
            eth_refclk_from_pll = True,
        )
        self.qpll.enable_pll_refclk()

        # SoCMini ----------------------------------------------------------------------------------

        SoCMini.__init__(self, platform,
            clk_freq      = sys_clk_freq,
            ident         = "LiteX-WR-NIC on Acorn Baseboard Mini.",
            ident_version = True,
        )

        # UART -------------------------------------------------------------------------------------

        self.uart = UARTShared(pads=platform.request("serial"), sys_clk_freq=sys_clk_freq)

        # JTAGBone ---------------------------------------------------------------------------------

        self.add_jtagbone()
        platform.add_period_constraint(self.jtagbone_phy.cd_jtag.clk, 1e9/20e6)
        platform.add_false_path_constraints(self.jtagbone_phy.cd_jtag.clk, self.crg.cd_sys.clk)

        # PCIe PHY ---------------------------------------------------------------------------------

        if with_pcie:
            self.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1"),
                data_width  = 64,
                bar0_size   = 0x20000,
                with_ptm    = True,
                refclk_freq = 100e6,
            )
            self.pcie_phy.update_config({
                "Base_Class_Menu"          : "Network_controller",
                "Sub_Class_Interface_Menu" : "Ethernet_controller",
                "Class_Code_Base"          : "02",
                "Class_Code_Sub"           : "00",
            })
            self.comb += ClockSignal("refclk_pcie").eq(self.pcie_phy.pcie_refclk)
            self.pcie_phy.use_external_qpll(qpll_channel=self.qpll.get_channel("pcie"))
            platform.add_period_constraint(self.crg.cd_sys.clk, 1e9/sys_clk_freq)
            platform.toolchain.pre_placement_commands.append("reset_property LOC [get_cells -hierarchical -filter {{NAME=~pcie_s7/*gtp_channel.gtpe2_channel_i}}]")
            platform.toolchain.pre_placement_commands.append("set_property LOC GTPE2_CHANNEL_X0Y7 [get_cells -hierarchical -filter {{NAME=~pcie_s7/*gtp_channel.gtpe2_channel_i}}]")

            # PCIe <-> Sys-Clk false paths.
            false_paths = [
                ("{{*s7pciephy_clkout0}}", "sys_clk"),
                ("{{*s7pciephy_clkout1}}", "sys_clk"),
                ("{{*s7pciephy_clkout3}}", "sys_clk"),
                ("{{*s7pciephy_clkout0}}", "{{*s7pciephy_clkout1}}")
            ]
            for clk0, clk1 in false_paths:
                platform.toolchain.pre_placement_commands.append(f"set_false_path -from [get_clocks {clk0}] -to [get_clocks {clk1}]")
                platform.toolchain.pre_placement_commands.append(f"set_false_path -from [get_clocks {clk1}] -to [get_clocks {clk0}]")

        # White Rabbit -----------------------------------------------------------------------------

        if with_white_rabbit:
            # Core Instance.
            # --------------
            self.add_wr_core(
                # CPU.
                cpu_firmware    = white_rabbit_cpu_firmware,

                # Board name.
                board_name       = "SAWR",

                # SFP.
                sfp_pads        = platform.request("sfp",     white_rabbit_sfp_connector),
                sfp_i2c_pads    = platform.request("sfp_i2c", white_rabbit_sfp_connector),
                sfp_tx_polarity = 0, # Inverted on Acorn and on baseboard.
                sfp_rx_polarity = 1, # Inverted on Acorn.

                # QPLL.
                qpll            = self.qpll,
                with_ext_clk    = False,

                # Serial.
                serial_pads     = self.uart.shared_pads,
            )
            self.add_sources()

            # RefClk MMCM Phase Shift.
            # ------------------------
            self.refclk_mmcm_ps_gen = PSGen(
                 cd_psclk    = "clk200",
                 cd_sys      = "wr",
                 ctrl_size   = 16,
                 )
            self.comb += [
                self.refclk_mmcm_ps_gen.ctrl_data.eq(self.dac_refclk_data),
                self.refclk_mmcm_ps_gen.ctrl_load.eq(self.dac_refclk_load),
                self.crg.refclk_mmcm.psen.eq(self.refclk_mmcm_ps_gen.psen),
                self.crg.refclk_mmcm.psincdec.eq(self.refclk_mmcm_ps_gen.psincdec),
            ]

            # DMTD MMCM Phase Shift.
            # ----------------------
            self.dmtd_mmcm_ps_gen = PSGen(
                 cd_psclk    = "clk200",
                 cd_sys      = "wr",
                 ctrl_size   = 16,
                 )
            self.comb += [
                self.dmtd_mmcm_ps_gen.ctrl_data.eq(self.dac_dmtd_data),
                self.dmtd_mmcm_ps_gen.ctrl_load.eq(self.dac_dmtd_load),
                self.crg.dmtd_mmcm.psen.eq(self.dmtd_mmcm_ps_gen.psen),
                self.crg.dmtd_mmcm.psincdec.eq(self.dmtd_mmcm_ps_gen.psincdec),
            ]

            # Timings Constraints.
            # --------------------
            platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-123]") # FIXME: Add 10MHz Ext Clk.
            platform.add_platform_command("create_clock -name wr_txoutclk -period 16.000 [get_pins -hierarchical *gtpe2_i/TXOUTCLK]")
            platform.add_platform_command("create_clock -name wr_rxoutclk -period 16.000 [get_pins -hierarchical *gtpe2_i/RXOUTCLK]")

            # Leds.
            # -----
            self.comb += [
                # Leds Output.
                platform.request("user_led", 0).eq(~self.led_link),
                platform.request("user_led", 1).eq(~self.led_act),
                platform.request("user_led", 2).eq(~self.led_pps),
            ]

            # GPIOs.
            # ------
            self.comb += [
                # PPS/Clk Output.
                platform.request("pps_out").eq(self.pps_out),
                platform.request("wr_clk_out").eq(ClockSignal("wr")),
            ]

            # White Rabbit Ethernet PHY (over White Rabbit Fabric) ---------------------------------

            self.ethphy0 = LiteEthPHYWRGMII(
                wrf_stream2wb = self.wrf_stream2wb,
                wrf_wb2stream = self.wrf_wb2stream,
            )

        # PCIe NIC ---------------------------------------------------------------------------------

        if with_pcie and with_white_rabbit:
            self.add_pcie_nic(pcie_phy=self.pcie_phy, eth_phys=[self.ethphy0], with_timing_constraints=False)
            self.add_pcie_ptm()

        # Etherbone --------------------------------------------------------------------------------

        if (not with_pcie) and with_white_rabbit:
            self.add_etherbone(phy=self.ethphy0, data_width=8, with_timing_constraints=False)

        # Time Generator ---------------------------------------------------------------------------

        if with_pcie and with_white_rabbit:
            # Time Generator.
            self.time_generator = TimeGenerator(
                clk_domain = "wr",
                clk_freq   = 62.5e6,
            )

            # Connect White Rabbit Time Interface to TimeGenerator's Sync Interface.
            self.comb += [
                self.time_generator.time_sync.eq(self.pps_out_pulse),
                self.time_generator.time_seconds.eq(self.tm_seconds),
            ]

            # Connect TimeGenerator's Time to PCIe PTM.
            self.comb += [
                self.ptm_requester.time_clk.eq(ClockSignal("wr")),
                self.ptm_requester.time_rst.eq(ResetSignal("wr")),
                self.ptm_requester.time.eq(self.time_generator.time)
            ]

        # Timing Constraints -----------------------------------------------------------------------

        asynchronous_clk_domains = [
            self.crg.cd_sys.clk,
            self.crg.cd_clk_62m5_dmtd.clk,
            self.crg.cd_clk_125m_gtp.clk,
            self.crg.cd_clk10m_in.clk,
            self.crg.cd_clk62m5_in.clk,
            "wr_txoutclk",
            "wr_rxoutclk",
        ]
        if with_white_rabbit:
            asynchronous_clk_domains += []

        platform.add_false_path_constraints(*asynchronous_clk_domains)

        # Clk Measurement (Debug) ------------------------------------------------------------------

        self.clk_measurement = MultiClkMeasurement(clks={
            "clk0" : ClockSignal("sys"),
            "clk1" : ClockSignal("clk_62m5_dmtd"),
            "clk2" : ClockSignal("clk_125m_gtp"),
            "clk3" : ClockSignal("clk10m_in"),
            "clk4" : ClockSignal("clk62m5_in"),
        })

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX-WR-NIC on Acorn Baseboard Mini.")

    # Build/Load/Flash Arguments.
    # ---------------------------
    parser.add_argument("--build", action="store_true", help="Build bitstream.")
    parser.add_argument("--load",  action="store_true", help="Load bitstream.")
    parser.add_argument("--flash", action="store_true", help="Flash bitstream.")

    # Probes.
    # -------
    parser.add_argument("--with-wishbone-fabric-interface-probe", action="store_true")
    parser.add_argument("--with-wishbone-slave-probe",            action="store_true")
    parser.add_argument("--with-dac-vcxo-probe",                  action="store_true")
    parser.add_argument("--with-time-pps-probe",                  action="store_true")

    args = parser.parse_args()

    # Build Firmware.
    # ---------------
    if args.build:
        print("Building firmware...")
        r = os.system("cd litex_wr_nic/firmware && ./build.py --target acorn")
        if r != 0:
            raise RuntimeError("Firmware build failed.")

    # Build SoC/Gateware (with integrated Firmware).
    # ----------------------------------------------
    soc = BaseSoC()
    if args.with_wishbone_fabric_interface_probe:
        soc.add_wishbone_fabric_interface_probe()
    if args.with_wishbone_slave_probe:
        soc.add_wishbone_slave_probe()
    if args.with_dac_vcxo_probe:
        soc.add_dac_vcxo_probe()
    if args.with_time_pps_probe:
        soc.add_time_pps_probe()
    builder = Builder(soc, csr_csv="test/csr.csv")
    builder.build(run=args.build)

    # Generate PCIe C Headers.
    # ------------------------
    generate_litepcie_software_headers(soc, "litex_wr_nic/software/kernel")

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
