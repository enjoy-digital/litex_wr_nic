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

from spec_a7_platform import *

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

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_white_rabbit=True):
        self.rst              = Signal()
        self.cd_sys           = ClockDomain()
        self.cd_refclk_pcie   = ClockDomain()
        self.cd_refclk_eth    = ClockDomain()
        self.cd_clk_125m_gtp  = ClockDomain()
        self.cd_clk_62m5_dmtd = ClockDomain()
        self.cd_clk10m_in     = ClockDomain()
        self.cd_clk62m5_in    = ClockDomain()

        # # #

        # Sys PLL (Free-Running from clk125).
        # ----------------------------------
        clk125m_oe = platform.request("clk125m_oe")
        clk125m    = platform.request("clk125m")
        self.comb += clk125m_oe.eq(1)

        self.pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk125m, 125e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, margin=0)

        # RefClk Input (125MHz from 25MHz VCXO x 5 (AD9516)).
        # ---------------------------------------------------
        refclk125m_pads = platform.request("refclk125m")
        refclk125m_se   = Signal()
        self.specials += Instance("IBUFDS_GTE2",
            i_CEB = 0,
            i_I   = refclk125m_pads.p,
            i_IB  = refclk125m_pads.n,
            o_O   = refclk125m_se,
        )
        self.comb += self.cd_clk_125m_gtp.clk.eq(refclk125m_se)
        self.comb += self.cd_refclk_eth.clk.eq(refclk125m_se)

        # DMTD PLL (62.5MHz from VCXO).
        # -----------------------------
        clk62m5_dmtd_pads = platform.request("clk62m5_dmtd")
        self.comb += self.cd_clk_62m5_dmtd.clk.eq(clk62m5_dmtd_pads)

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

        # Sync-In Parameters.
        # -------------------
        pps_in_macro_delay_default  = 62500000, # 16ns taps (Up to 2**32-1 taps).

        # Sync-Out Parameters.
        # --------------------
        bypass_pps_out_coarse_delay  = False,
        # PPS Out (Adjusted over JTAGBone with test/test_delay.py).
        pps_out_macro_delay_default  = 62499998, # 16ns taps (Up to 2**32-1 taps).
        pps_out_coarse_delay_default =        1, #  2ns taps (64 taps).
        pps_out_fine_delay_default   =      100, # 11ps taps (512 taps).

        # Clk10M Out (Adjusted over JTAGBone with test/test_delay.py).
        clk10m_out_macro_delay_default  = 6250000, # 16ns taps (Up to 2**32-1 taps).
        clk10m_out_coarse_delay_default =       1, #  2ns taps (64 taps).
        clk10m_out_fine_delay_default   =     200, # 11ps taps (512 taps).

        # Sync-In Parameters.
        # -------------------
        with_sync_in_pll = True,

        # RF-Out Parameters.
        # ------------------
        with_rf_out = True,
    ):
        # Platform ---------------------------------------------------------------------------------

        platform      = Platform(variant="xc7a50t")
        platform.name = "spec_a7_wr_nic"

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
            ident         = "LiteX-WR-NIC on SPEC-A7.",
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
            platform.toolchain.pre_placement_commands.append("set_property LOC GTPE2_CHANNEL_X0Y0 [get_cells -hierarchical -filter {{NAME=~pcie_s7/*gtp_channel.gtpe2_channel_i}}]")

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
            # White Rabbit Core.
            # ------------------
            self.add_wr_core(
                # CPU.
                cpu_firmware     = white_rabbit_cpu_firmware,

                # Board name.
                board_name       = "SPA7",

                # SFP.
                sfp_pads         = platform.request("sfp",     white_rabbit_sfp_connector),
                sfp_i2c_pads     = platform.request("sfp_i2c", white_rabbit_sfp_connector),
                sfp_tx_polarity  = 0, # Not Inverted.
                sfp_rx_polarity  = 0, # Not Inverted.
                sfp_disable_pads = platform.request("sfp_disable", white_rabbit_sfp_connector),
                sfp_fault_pads   = platform.request("sfp_fault",   white_rabbit_sfp_connector),
                sfp_los_pads     = platform.request("sfp_los",     white_rabbit_sfp_connector),
                sfp_det_pads     = platform.request("sfp_det",     white_rabbit_sfp_connector),

                # QPLL.
                qpll             = self.qpll,
                with_ext_clk     = True,

                # Serial.
                serial_pads      = self.uart.shared_pads,

                # Flash.
                flash_pads       = platform.request("flash"),

                # Temp 1Wire.
                temp_1wire_pads  = platform.request("temp_1wire"),
            )
            self.add_sources()

            # Pads.
            # -----
            dac_refclk_pads  = platform.request("dac_refclk")
            dac_dmtd_pads    = platform.request("dac_dmtd")
            clk10m_in_pads   = platform.request("clk10m_in")
            clk62m5_in_pads  = platform.request("clk62m5_in")
            pps_in_pads      = platform.request("pps_in")

            # White Rabbit RefClk AD9516 PLL Driver.
            # --------------------------------------
            self.refclk_pll = AD9516PLL(
                platform   = platform,
                pads       = platform.request("pll"),
                config     = AD9516_MAIN_CONFIG,
                name       = "main",
                clk_domain = "sys",
            )

            # White Rabbit RefClk / DMTD DAC Drivers.
            # ---------------------------------------

            # RefClk DAC.
            self.refclk_dac = AD5683RDAC(platform,
                pads  = dac_refclk_pads,
                load  = self.dac_refclk_load,
                value = self.dac_refclk_data,
                gain  = 2, # 2 for 0-3V range to be able to accelerate enough RefClk, not working with 1.
            )

            # DMTD DAC.
            self.dmtd_dac = AD5683RDAC(platform,
                pads  = dac_dmtd_pads,
                load  = self.dac_dmtd_load,
                value = self.dac_dmtd_data,
                gain  = 1,
            )

            # White Rabbit Clk-In.
            # --------------------

            # White Rabbit Sync-In AD9516 PLL Driver.
            if with_sync_in_pll:
                self.sync_in_pll = AD9516PLL(
                    platform   = platform,
                    pads       = platform.request("sync_in_pll"),
                    config     = AD9516_EXT_CONFIG,
                    name       = "sync_in",
                    clk_domain = "sys",
                )

            # Clk10m In Logic.
            self.specials += DifferentialInput(
                i_p = clk10m_in_pads.p,
                i_n = clk10m_in_pads.n,
                o   = self.crg.cd_clk10m_in.clk,
            )

            # Clk62m5 In Logic.
            self.specials += DifferentialInput(
                i_p = clk62m5_in_pads.p,
                i_n = clk62m5_in_pads.n,
                o   = self.crg.cd_clk62m5_in.clk,
            )

            # White Rabbit PPS-In and Macro Delay.
            # ------------------------------------
            pps_in             = Signal()
            pps_in_d           = Signal()
            pps_in_pulse       = Signal()
            pps_in_macro_delay = Signal()

            self.comb += platform.request("pps_in_term_en").eq(1) # CHECKME: Make it configurable?

            self.specials += MultiReg(pps_in_pads, pps_in, odomain="wr")
            self.sync.wr += pps_in_d.eq(pps_in)
            self.comb += pps_in_pulse.eq(pps_in & ~pps_in_d)

            self.pps_in_macro_delay = MacroDelay(
                pulse_i = pps_in_pulse,
                pulse_o = pps_in_macro_delay,
                clk_domain    = "wr",
                default_delay = pps_in_macro_delay_default,
            )
            self.comb += self.pps_in.eq(pps_in_macro_delay)

            # Timings Constraints.
            # --------------------
            platform.add_platform_command("create_clock -name wr_txoutclk -period 16.000 [get_pins -hierarchical *gtpe2_i/TXOUTCLK]")
            platform.add_platform_command("create_clock -name wr_rxoutclk -period 16.000 [get_pins -hierarchical *gtpe2_i/RXOUTCLK]")

            # White Rabbit Ethernet PHY (over White Rabbit Fabric) ---------------------------------

            self.ethphy0 = LiteEthPHYWRGMII(
                wrf_stream2wb = self.wrf_stream2wb,
                wrf_wb2stream = self.wrf_wb2stream,
            )

            # White Rabbit Sync-Out ----------------------------------------------------------------

            # PPS Free-Running.
            # -----------------
            self.pps_freerun = ClockDomainsRenamer("wr")(WaitTimer(int(62.5e6 - 1)))
            self.comb += self.pps_freerun.wait.eq(~self.pps_freerun.done)

            # PPS Out Valid.
            # --------------
            # PPS is considered inactive if no PPS pulse from WR for 2s.
            pps_out_valid        = Signal()
            pps_out_active_timer = WaitTimer(2.0*62.5e6)
            pps_out_active_timer = ClockDomainsRenamer("wr")(pps_out_active_timer)
            self.submodules += pps_out_active_timer
            self.comb += [
                pps_out_active_timer.wait.eq(~self.pps_out_pulse),
                pps_out_valid.eq(~pps_out_active_timer.done),
            ]

            # PPS WR / Free-Running Selection.
            # --------------------------------
            pps_out_pulse_sel = Signal()
            self.comb += [
                # Use PPS from WR when active.
                If(pps_out_valid,
                    pps_out_pulse_sel.eq(self.pps_out_pulse)
                # Else Switch back to Free-Running PPS.
                ).Else(
                    pps_out_pulse_sel.eq(self.pps_freerun.done)
                )
            ]

            # Sync-Out PLL.
            # -------------
            self.cd_wr8x     = ClockDomain()
            self.syncout_pll = syncout_pll = S7MMCM(speedgrade=-2)
            self.comb += syncout_pll.reset.eq(ResetSignal("wr"))
            syncout_pll.register_clkin(ClockSignal("wr"), 62.5e6)
            syncout_pll.create_clkout(self.cd_wr8x, 500e6, margin=0, phase=0)

            # Clk10M SMA Out.
            # ---------------

            # Clk10M Macro Delay.
            clk10m_out_macro_delay  = Signal()
            self.clk10m_macro_delay = MacroDelay(
                pulse_i = pps_out_pulse_sel,
                pulse_o = clk10m_out_macro_delay,
                clk_domain    = "wr",
                default_delay = clk10m_out_macro_delay_default,
            )

            # Clk10M Generator.
            clk10m_out_gen  = Signal(8)
            self.clk10m_gen = Clk10MGenerator(
                pulse_i  = clk10m_out_macro_delay,
                clk10m_o = clk10m_out_gen,
                clk_domain = "wr",
            )

            # Clk10M Coarse Delay.
            clk10m_out_coarse_delay      = Signal()
            self.clk10m_out_coarse_delay = CoarseDelay(
                rst = ~syncout_pll.locked,
                i   = clk10m_out_gen,
                o   = clk10m_out_coarse_delay,
                clk_domain = "wr",
                clk_cycles = 8, # 64-taps.
                default_delay = clk10m_out_coarse_delay_default,
            )

            # Clk10M Out.
            clk10m_out_pads = platform.request("clk10m_out")
            self.specials += DifferentialOutput(
                i   = clk10m_out_coarse_delay,
                o_p = clk10m_out_pads.p,
                o_n = clk10m_out_pads.n,
            )

            # PPS SMA Out.
            # ------------

            # PPS Macro Delay.
            pps_out_macro_delay      = Signal()
            self.pps_out_macro_delay = MacroDelay(
                pulse_i = pps_out_pulse_sel,
                pulse_o = pps_out_macro_delay,
                clk_domain    = "wr",
                default_delay = pps_out_macro_delay_default,
            )

            # PPS Generator.
            pps_out_gen = Signal()
            self.pps_out_gen = PPSGenerator(
                i = pps_out_macro_delay,
                o = pps_out_gen,
                clk_domain = "wr",
                clk_freq   = int(62.5e6),
                duty_cycle = 20/100, # 20% High / 80% Low PPS.
            )

            # PPS Coarse Delay.
            pps_out_coarse_delay      = Signal()
            if not bypass_pps_out_coarse_delay:
                self.pps_out_coarse_delay = CoarseDelay(
                    rst = ~syncout_pll.locked,
                    i   = pps_out_gen,
                    o   = pps_out_coarse_delay,
                    clk_domain = "wr",
                    clk_cycles = 8, # 64-taps.
                    default_delay = pps_out_coarse_delay_default,
                )

            # PPS Out.
            pps_out_pads = platform.request("pps_out")
            self.specials += DifferentialOutput(
                i   = pps_out_gen if bypass_pps_out_coarse_delay else pps_out_coarse_delay,
                o_p = pps_out_pads.p,
                o_n = pps_out_pads.n,
            )


            # Fine Delay (Clk10M & PPS Out).
            # ------------------------------
            self.fine_delay = FineDelay(
                pads           = platform.request("fine_delay"),
                sys_clk_freq   = sys_clk_freq,
                default_delays = [
                    clk10m_out_fine_delay_default,
                    pps_out_fine_delay_default,
                ],
            )

            # FrontPanel Leds.
            # ----------------
            self.comb += [
                platform.request("clk10m_out_led").eq(1),
                platform.request("pps_out_led").eq(pps_out_gen),
                platform.request("act_out_led").eq(self.led_link & ~self.led_act)
            ]

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

        # RF Out (LMX2572) -------------------------------------------------------------------------
        # CHECKME: Connect SYNC if useful.

        if with_rf_out:
            rf_out_pll_pads = platform.request("rf_out_pll")
            rf_out_pll_pads.miso = Signal()
            self.rf_out_pll = SPIMaster(
                pads         = rf_out_pll_pads,
                data_width   = 24,
                sys_clk_freq = sys_clk_freq,
                spi_clk_freq = 5e6,
                mode         = "aligned",
            )

        # Timing Constraints -----------------------------------------------------------------------

        asynchronous_clk_domains = [
            self.crg.cd_sys.clk,
            self.crg.cd_clk_62m5_dmtd.clk,
            self.crg.cd_clk_125m_gtp.clk,
            self.crg.cd_clk10m_in.clk,
            self.crg.cd_clk62m5_in.clk,
            self.cd_wr8x.clk,
            "wr_txoutclk",
            "wr_rxoutclk",
        ]
        if with_white_rabbit:
            asynchronous_clk_domains += [self.fine_delay.cd_fine_delay.clk]

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
    parser = argparse.ArgumentParser(description="LiteX-WR-NIC on SPEC-A7.")

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
        r = os.system("cd litex_wr_nic/firmware && ./build.py")
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

    # Generate Bitstream.
    # -------------------
    if args.load or args.flash:
        os.system("python3 litex_wr_nic/gateware/xilinx-bitstream.py {bit_file} {bin_file}".format(
            bit_file = builder.get_bitstream_filename(mode="sram"),
            bin_file = builder.get_bitstream_filename(mode="flash"),
        ))

    # Load FPGA.
    # ----------
    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="flash"))

    # Flash FPGA.
    # -----------
    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0x0000_0000, builder.get_bitstream_filename(mode="flash"))
        prog.flash(0x002e_0000, "firmware/sdb-wrpc.bin")

if __name__ == "__main__":
    main()
