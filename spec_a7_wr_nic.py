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
from litex.build.io               import SDRTristate, SDROutput
from litex.build.io               import DifferentialInput, DifferentialOutput
from litex.build.openfpgaloader   import OpenFPGALoader

from litex.soc.interconnect.csr     import *
from litex.soc.interconnect         import stream
from litex.soc.interconnect         import wishbone

from litex.soc.integration.soc      import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.clock          import S7PLL, S7MMCM
from litex.soc.cores.led            import LedChaser
from litex.soc.cores.spi.spi_master import SPIMaster

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.software      import generate_litepcie_software_headers

from litescope import LiteScopeAnalyzer

from gateware.uart              import UARTShared
from gateware.soc               import LiteXWRNICSoC
from gateware.time              import TimeGenerator
from gateware.qpll              import SharedQPLL
from gateware.wb_clock_crossing import WishboneClockCrossing
from gateware.wrf_stream2wb     import Stream2Wishbone
from gateware.wrf_wb2stream     import Wishbone2Stream
from gateware.ad5683r.core      import AD5683RDAC
from gateware.ad9516.core       import AD9516PLL, AD9516_MAIN_CONFIG, AD9516_EXT_CONFIG
from gateware.measurement       import MultiClkMeasurement
from gateware.delay.core        import MacroDelay, CoarseDelay, FineDelay
from gateware.pps               import PPSGenerator
from gateware.clk10m            import Clk10MGenerator
from gateware.nic.phy           import LiteEthPHYWRGMII

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_white_rabbit=True):
        self.rst            = Signal()
        self.cd_sys         = ClockDomain()
        self.cd_refclk_pcie = ClockDomain()
        self.cd_refclk_eth  = ClockDomain()
        if with_white_rabbit:
            self.cd_clk_125m_gtp  = ClockDomain()
            self.cd_clk_62m5_dmtd = ClockDomain()
        self.cd_clk10m_in  = ClockDomain()
        self.cd_clk62m5_in = ClockDomain()
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

        # False Paths.
        # ------------
        if with_white_rabbit:
            platform.add_false_path_constraints(
                pll.clkin,
                self.cd_sys.clk,
                self.cd_clk_62m5_dmtd.clk,
                self.cd_clk_125m_gtp.clk,
                self.cd_clk10m_in.clk,
                self.cd_clk62m5_in.clk,
            )
        else:
            platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(LiteXWRNICSoC):
    def __init__(self, sys_clk_freq=125e6,
        # PCIe Parameters.
        # ----------------
        with_pcie     = True,
        with_pcie_ptm = True,
        with_pcie_nic = True,

        # White Rabbit Parameters.
        # ------------------------
        with_white_rabbit          = True,
        white_rabbit_sfp_connector = 0,
        white_rabbit_cpu_firmware  = "firmware/spec_a7_wrc.bram",

        # Sync-Out Parameters.
        # --------------------
        # PPS Out (Adjusted over JTAGBone with test/test_delay.py).
        pps_out_macro_delay_default  = 62499998, # 16ns taps (Up to 2**32-1 taps).
        pps_out_coarse_delay_default =        1, #  2ns taps (64 taps).
        pps_out_fine_delay_default   =      100, # 11ps taps (512 taps).

        # Clk10M Out (Adjusted over JTAGBone with test/test_delay.py).
        clk10m_out_macro_delay_default  = 6250000, # 16ns taps (Up to 2**32-1 taps).
        clk10m_out_coarse_delay_default =      10, #  2ns taps (64 taps).
        clk10m_out_fine_delay_default   =     100, # 11ps taps (512 taps).

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

        # PCIe -------------------------------------------------------------------------------------

        if with_pcie:
            self.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1"),
                data_width  = 64,
                bar0_size   = 0x20000,
                with_ptm    = with_pcie_ptm,
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


            if not with_pcie_nic:
                self.add_pcie(phy=self.pcie_phy,
                    ndmas                = 1,
                    address_width        = 64,
                    with_ptm             = True,
                    max_pending_requests = 2,
                )

        # White Rabbit -----------------------------------------------------------------------------

        if with_white_rabbit:
            # Clks.
            # -----
            self.cd_wr = ClockDomain("wr")

            # Pads.
            # -----
            dac_refclk_pads  = platform.request("dac_refclk")
            dac_dmtd_pads    = platform.request("dac_dmtd")
            sfp_disable_pads = platform.request("sfp_disable", white_rabbit_sfp_connector)
            sfp_fault_pads   = platform.request("sfp_fault",   white_rabbit_sfp_connector)
            sfp_los_pads     = platform.request("sfp_los",     white_rabbit_sfp_connector)
            sfp_pads         = platform.request("sfp",         white_rabbit_sfp_connector)
            sfp_i2c_pads     = platform.request("sfp_i2c",     white_rabbit_sfp_connector)
            sfp_det_pads     = platform.request("sfp_det",     white_rabbit_sfp_connector)
            temp_1wire_pads  = platform.request("temp_1wire")
            flash_pads       = platform.request("flash")
            flash_clk        = Signal()
            clk10m_in_pads   = platform.request("clk10m_in")
            clk10m_in        = Signal()
            clk62m5_in_pads  = platform.request("clk62m5_in")
            clk62m5_in       = Signal()
            pps_in_pads      = platform.request("pps_in")
            pps_in           = Signal()

            # Temp 1-Wire specific logic.
            temp_1wire_oe_n = Signal()
            temp_1wire_i    = Signal()
            self.specials += SDRTristate(
                io = temp_1wire_pads,
                o  = Constant(0b0, 1),
                oe = ~temp_1wire_oe_n,
                i  = temp_1wire_i,
            )

            # Flash specific logic.
            self.specials += Instance("STARTUPE2",
                i_CLK       = 0,
                i_GSR       = 0,
                i_GTS       = 0,
                i_KEYCLEARB = 0,
                i_PACK      = 0,
                i_USRCCLKO  = flash_clk,
                i_USRCCLKTS = 0,
                i_USRDONEO  = 1,
                i_USRDONETS = 1,
            )

            # Signals.
            # --------
            led_pps         = Signal()
            led_link        = Signal()
            led_act         = Signal()
            dac_refclk_load = Signal()
            dac_refclk_data = Signal(16)
            dac_dmtd_load   = Signal()
            dac_dmtd_data   = Signal(16)
            pps_in          = Signal()
            pps_out_valid   = Signal()
            pps_out         = Signal()
            pps_out_pulse   = Signal()
            tm_link_up      = Signal()
            tm_time_valid   = Signal()
            tm_seconds      = Signal(40)
            tm_cycles       = Signal(28)

            # White Rabbit Fabric Interface.
            # ------------------------------
            wrf_src = wishbone.Interface(data_width=16, address_width=2, adressing="byte")
            wrf_snk = wishbone.Interface(data_width=16, address_width=2, adressing="byte")

            self.wrf_stream2wb = wrf_stream2wb = Stream2Wishbone(  cd_to="wr")
            self.wrf_wb2stream = wrf_wb2stream = Wishbone2Stream(cd_from="wr")

            # White Rabbit Slave Interface.
            # -----------------------------
            self.wb_slave_sys = wb_slave_sys = wishbone.Interface(data_width=32, address_width=32, adressing="byte")
            self.wb_slave_wr  = wb_slave_wr  = wishbone.Interface(data_width=32, address_width=32, adressing="byte")
            self.bus.add_slave(name="wr_wb_slave", slave=wb_slave_sys, region=SoCRegion(
                 origin = 0x2000_0000,
                 size   = 0x0100_0000,
             ))
            self.submodules += WishboneClockCrossing(platform,
                wb_from = wb_slave_sys,
                cd_from = "sys",
                wb_to   = wb_slave_wr,
                cd_to   = "wr",
            )

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
                load  = dac_refclk_load,
                value = dac_refclk_data,
                gain  = 2, # 2 for 0-3V range to be able to accelerate enough RefClk, not working with 1.
            )

            # DMTD DAC.
            self.dmtd_dac = AD5683RDAC(platform,
                pads  = dac_dmtd_pads,
                load  = dac_dmtd_load,
                value = dac_dmtd_data,
                gain  = 1,
            )

            # White Rabbit Sync-In -----------------------------------------------------------------

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
                o   = clk10m_in,
            )
            self.comb += self.crg.cd_clk10m_in.clk.eq(clk10m_in)

            # Clk62m5 In Logic.
            self.specials += DifferentialInput(
                i_p = clk62m5_in_pads.p,
                i_n = clk62m5_in_pads.n,
                o   = clk62m5_in,
            )
            self.comb += self.crg.cd_clk62m5_in.clk.eq(clk62m5_in)

            # PPS In Logic.
            self.comb += platform.request("pps_in_term_en").eq(1)
            self.comb += pps_in.eq(pps_in_pads)

            # White Rabbit Core Instance.
            # ---------------------------
            self.specials += Instance("xwrc_board_spec_a7_wrapper",
                # Parameters.
                p_g_dpram_initf       = os.path.abspath(white_rabbit_cpu_firmware),
                p_g_dpram_size        = 131072/4,
                p_txpolarity          = 0, # Not Inverted.
                p_rxpolarity          = 0, # Not Inverted.

                # Clocks/resets.
                i_areset_n_i          = ~ResetSignal("sys"),
                i_clk_62m5_dmtd_i     = ClockSignal("clk_62m5_dmtd"),
                i_clk_125m_gtp_i      = ClockSignal("clk_125m_gtp"),
                i_clk_10m_ext_i       = ClockSignal("clk10m_in"),
                o_clk_62m5_sys_o      = ClockSignal("wr"),
                o_rst_62m5_sys_o      = ResetSignal("wr"),

                # DAC RefClk Interface.
                o_dac_refclk_load     = dac_refclk_load,
                o_dac_refclk_data     = dac_refclk_data,

                # DAC DMTD Interface.
                o_dac_dmtd_load       = dac_dmtd_load,
                o_dac_dmtd_data       = dac_dmtd_data,

                # SFP Interface.
                o_sfp_txp_o           = sfp_pads.txp,
                o_sfp_txn_o           = sfp_pads.txn,
                i_sfp_rxp_i           = sfp_pads.rxp,
                i_sfp_rxn_i           = sfp_pads.rxn,
                i_sfp_det_i           = sfp_det_pads,
                io_sfp_sda            = sfp_i2c_pads.sda,
                io_sfp_scl            = sfp_i2c_pads.scl,
                i_sfp_tx_fault_i      = sfp_fault_pads,
                i_sfp_tx_los_i        = sfp_los_pads,
                o_sfp_tx_disable_o    = sfp_disable_pads,

                # One-Wire Interface.
                i_onewire_i           = temp_1wire_i,
                o_onewire_oen_o       = temp_1wire_oe_n,

                # UART Interface.
                i_uart_rxd_i          = self.uart.wr_pads.rx,
                o_uart_txd_o          = self.uart.wr_pads.tx,

                # SPI Flash Interface.
                o_spi_sclk_o          = flash_clk,
                o_spi_ncs_o           = flash_pads.cs_n,
                o_spi_mosi_o          = flash_pads.mosi,
                i_spi_miso_i          = flash_pads.miso,

                # PPS / Leds.
                o_pps_valid_o         = Open(),
                i_pps_ext_i           = pps_in,
                o_pps_csync_o         = pps_out_pulse,
                o_pps_p_o             = pps_out,
                o_pps_led_o           = led_pps,
                o_led_link_o          = led_link,
                o_led_act_o           = led_act,

                # QPLL Interface (for GTPE2_Common Sharing).
                o_gt0_ext_qpll_reset  = self.qpll.get_channel("eth").reset,
                i_gt0_ext_qpll_clk    = self.qpll.get_channel("eth").clk,
                i_gt0_ext_qpll_refclk = self.qpll.get_channel("eth").refclk,
                i_gt0_ext_qpll_lock   = self.qpll.get_channel("eth").lock,

                # Wishbone Slave Interface (MMAP).
                i_wb_slave_cyc        = wb_slave_wr.cyc,
                i_wb_slave_stb        = wb_slave_wr.stb,
                i_wb_slave_we         = wb_slave_wr.we,
                i_wb_slave_adr        = Cat(Signal(2), (wb_slave_wr.adr & 0x00ff_ffff)),
                i_wb_slave_sel        = wb_slave_wr.sel,
                i_wb_slave_dat_i      = wb_slave_wr.dat_w,
                o_wb_slave_dat_o      = wb_slave_wr.dat_r,
                o_wb_slave_ack        = wb_slave_wr.ack,
                o_wb_slave_err        = wb_slave_wr.err,
                o_wb_slave_rty        = Open(),
                o_wb_slave_stall      = Open(),

                # Wishbone Fabric Source Interface.
                o_wrf_src_adr         = wrf_wb2stream.bus.adr,
                o_wrf_src_dat         = wrf_wb2stream.bus.dat_w,
                o_wrf_src_cyc         = wrf_wb2stream.bus.cyc,
                o_wrf_src_stb         = wrf_wb2stream.bus.stb,
                o_wrf_src_we          = wrf_wb2stream.bus.we,
                o_wrf_src_sel         = wrf_wb2stream.bus.sel,

                i_wrf_src_ack         = wrf_wb2stream.bus.ack,
                i_wrf_src_stall       = 0, # Not Used.
                i_wrf_src_err         = wrf_wb2stream.bus.err,
                i_wrf_src_rty         = 0, # Not Used.

                # Wishbone Fabric Sink Interface.
                i_wrf_snk_adr         = wrf_stream2wb.bus.adr,
                i_wrf_snk_dat         = wrf_stream2wb.bus.dat_w,
                i_wrf_snk_cyc         = wrf_stream2wb.bus.cyc,
                i_wrf_snk_stb         = wrf_stream2wb.bus.stb,
                i_wrf_snk_we          = wrf_stream2wb.bus.we,
                i_wrf_snk_sel         = wrf_stream2wb.bus.sel,

                o_wrf_snk_ack         = wrf_stream2wb.bus.ack,
                o_wrf_snk_stall       = Open(), # Not Used.
                o_wrf_snk_err         = wrf_stream2wb.bus.err,
                o_wrf_snk_rty         = Open(), # Not Used.

                # Time.
                o_tm_link_up_o        = tm_link_up,
                o_tm_time_valid_o     = tm_time_valid,
                o_tm_tai_o            = tm_seconds,
                o_tm_cycles_o         = tm_cycles,
            )
            self.add_sources()

            # FIXME: Improve constraints.
            platform.add_platform_command("create_clock -name wr_txoutclk -period 16.000 [get_pins -hierarchical *gtpe2_i/TXOUTCLK]")
            platform.add_platform_command("create_clock -name wr_rxoutclk -period 16.000 [get_pins -hierarchical *gtpe2_i/RXOUTCLK]")
#            def add_wr_false_paths(platform, white_rabbit_clocks):
#                design_clocks = [
#                    "sys_clk",
#                    #"clk125m_p",
#                    #"refclk125m_p",
#                    #"clk62m5_dmtd",
#                    "jtag_clk",
#                    #"pcie_x1_clk_p",
#                    #"clk10m_in_p",
#                    #"clk62m5_in_p",
#                ]
#                for design_clk in design_clocks:
#                    for wr_clk in white_rabbit_clocks:
#                        platform.add_platform_command(f"set_false_path -from [get_clocks {design_clk}] -to [get_clocks {wr_clk}]")
#                        platform.add_platform_command(f"set_false_path -from [get_clocks {wr_clk}] -to [get_clocks {design_clk}]")
#
#            add_wr_false_paths(platform, ["wr_txoutclk", "wr_rxoutclk"])

            # White Rabbit Ethernet PHY (over White Rabbit Fabric) ---------------------------------
            self.ethphy0 = LiteEthPHYWRGMII(wrf_stream2wb, wrf_wb2stream)

            if not with_pcie_nic:
                self.add_etherbone(phy=self.ethphy0, data_width=8, with_timing_constraints=False)
            else:
                self.add_pcie_nic(pcie_phy=self.pcie_phy, eth_phys=[self.ethphy0], with_timing_constraints=False)


            # White Rabbit Sync-Out ----------------------------------------------------------------

            # PPS Out Valid.
            # --------------
            # PPS is considered inactive if not PPS pulse from WR for 2s.
            pps_out_active_timer = WaitTimer(2.0*62.5e6)
            pps_out_active_timer = ClockDomainsRenamer("wr")(pps_out_active_timer)
            self.submodules += pps_out_active_timer
            self.comb += [
                pps_out_active_timer.wait.eq(~pps_out),
                pps_out_valid.eq(~pps_out_active_timer.done),
            ]

            # Sync-Out PLL.
            # -------------
            self.cd_wr8x      = ClockDomain()
            self.syncout_pll  = syncout_pll = S7MMCM(speedgrade=-2)
            self.comb += syncout_pll.reset.eq(ResetSignal("wr"))
            syncout_pll.register_clkin(ClockSignal("wr"), 62.5e6)
            syncout_pll.create_clkout(self.cd_wr8x, 500e6, margin=0, phase=0)

            # Clk10M SMA Out.
            # ---------------

            # Clk10M Macro Delay.
            clk10m_out_macro_delay = Signal()
            self.clk10m_macro_delay = MacroDelay(
                pulse_i = pps_out_pulse,
                pulse_o = clk10m_out_macro_delay,
                clk_domain    = "wr",
                default_delay = clk10m_out_macro_delay_default,
            )

            # Clk10M Generator.
            clk10m_out_gen = Signal()
            self.clk10m_gen = Clk10MGenerator(
                pulse_i  = clk10m_out_macro_delay,
                clk10m_o = clk10m_out_gen,
                clk_domain = "wr8x",
            )

            # Clk10M Coarse Delay.
            clk10m_out_coarse_delay = Signal()
            self.clk10m_out_coarse_delay = CoarseDelay(
                rst = ~syncout_pll.locked,
                i   = clk10m_out_gen & pps_out_valid,
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
            pps_out_macro_delay = Signal()
            self.pps_macro_delay = MacroDelay(
                pulse_i = pps_out_pulse,
                pulse_o = pps_out_macro_delay,
                clk_domain    = "wr",
                default_delay = pps_out_macro_delay_default,
            )

            # PPS Generator.
            pps_out_gen = Signal()
            self.pps_gen = PPSGenerator(
                i = pps_out_macro_delay,
                o = pps_out_gen,
                clk_domain = "wr",
                clk_freq   = int(62.5e6),
                duty_cycle = 20/100, # 20% High / 80% Low PPS.
            )

            # PPS Coarse Delay.
            pps_out_coarse_delay   = Signal()
            self.pps_out_coarse_delay = CoarseDelay(
                rst = ~syncout_pll.locked,
                i   = pps_out_gen & pps_out_valid,
                o   = pps_out_coarse_delay,
                clk_domain = "wr",
                clk_cycles = 8, # 64-taps.
                default_delay = pps_out_coarse_delay_default,
            )

            # PPS Out.
            pps_out_pads = platform.request("pps_out")
            self.specials += DifferentialOutput(
                i   = pps_out_coarse_delay,
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
                platform.request("clk10m_out_led").eq(pps_out_valid),
                platform.request("pps_out_led").eq(led_pps),
            ]

        # PCIe PTM ---------------------------------------------------------------------------------

        if with_pcie_ptm:
            assert with_pcie
            assert with_white_rabbit
            self.add_pcie_ptm()

            # Time Generator -----------------------------------------------------------------------

            self.time_generator = TimeGenerator(
                clk_domain = "wr",
                clk_freq   = 62.5e6,
            )

            # Connect White Rabbit Time Interface to TimeGenerator's Sync Interface.
            self.comb += [
                self.time_generator.time_sync.eq(pps_out_pulse),
                self.time_generator.time_seconds.eq(tm_seconds),
            ]

            # Connect TimeGenerator's Time to PCIe PTM.
            self.comb += [
                self.ptm_requester.time_clk.eq(ClockSignal("wr")),
                self.ptm_requester.time_rst.eq(ResetSignal("wr")),
                self.ptm_requester.time.eq(self.time_generator.time)
            ]


        # RF Out (LMX2572) -------------------------------------------------------------------

        if with_rf_out:

            # FIXME: Connect SYNC.

            rf_out_pll_pads = platform.request("rf_out_pll")
            rf_out_pll_pads.miso = Signal()
            self.rf_out_pll = SPIMaster(
                pads         = rf_out_pll_pads,
                data_width   = 24,
                sys_clk_freq = sys_clk_freq,
                spi_clk_freq = 5e6,
                mode         = "aligned",
            )
            self.rf_out_pll_sync = CSRStorage()
            self.comb += rf_out_pll_pads.sync.eq(self.rf_out_pll_sync.storage)

#            analyzer_signals = [
#                rf_out_pll_pads,
#                pps_out,
#                pps_out_pulse,
#                pps_in,
#            ]
#            self.analyzer = LiteScopeAnalyzer(analyzer_signals,
#                depth        = 1024,
#                clock_domain = "wr",
#                samplerate   = int(62.5e6),
#                register     = True,
#                csr_csv      = "test/analyzer.csv"
#            )

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
    parser.add_argument("--with-time-probe",                      action="store_true")

    args = parser.parse_args()

    # Build Firmware.
    # ---------------
    if args.build:
        print("Building firmware...")
        r = os.system("cd firmware && ./build.py")
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
    if args.with_time_probe:
        soc.add_time_probe()
    builder = Builder(soc, csr_csv="test/csr.csv")
    builder.build(run=args.build)

    # Generate PCIe C Headers.
    # ------------------------
    generate_litepcie_software_headers(soc, "software/kernel")

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
        prog.load_bitstream(builder.get_bitstream_filename(mode="flash"))

    # Flash FPGA.
    # -----------
    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0x0000_0000, builder.get_bitstream_filename(mode="flash"))
        prog.flash(0x002e_0000, "firmware/sdb-wrpc.bin")

if __name__ == "__main__":
    main()
