#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse
import subprocess

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from spec_a7_platform import *

from litex.build.generic_platform import *
from litex.build.io               import DifferentialInput
from litex.build.openfpgaloader   import OpenFPGALoader

from litex.soc.interconnect.csr     import *
from litex.soc.interconnect         import stream
from litex.soc.interconnect         import wishbone

from litex.soc.integration.soc      import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.clock import *
from litex.soc.cores.led   import LedChaser

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.frontend.ptm  import PCIePTMSniffer
from litepcie.frontend.ptm  import PTMCapabilities, PTMRequester
from litepcie.software      import generate_litepcie_software_headers

from litescope import LiteScopeAnalyzer

from gateware.wr_common     import wr_core_init, wr_core_files
from gateware.time          import TimeGenerator
from gateware.qpll          import SharedQPLL
from gateware.udp           import UDPPacketGenerator
from gateware.wrf_stream2wb import Stream2Wishbone
from gateware.wrf_wb2stream import Wishbone2Stream

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_white_rabbit=True, with_pcie=True, use_cfgm_clk=False):
        self.rst            = Signal()
        self.cd_sys         = ClockDomain()
        self.cd_refclk_pcie = ClockDomain()
        self.cd_refclk_eth  = ClockDomain()
        if with_white_rabbit:
            self.cd_clk_125m_dmtd = ClockDomain() # CHECKME/FIXME: Replace with appropriate clk.
            self.cd_clk_125m_gtp  = ClockDomain() # CHECKME/FIXME: Replace with appropriate clk.
            self.cd_clk_10m_ext   = ClockDomain( )# CHECKME/FIXME: Replace with appropriate clk.
        if with_pcie:
            self.cd_clk50 = ClockDomain()

        # # #

        # Clk/Rst.
        if use_cfgm_clk:
            # CFGM Clk ~65MHz.
            cfgm_clk      = Signal()
            cfgm_clk_freq = int(65e6)
            self.specials += Instance("STARTUPE2",
                i_CLK       = 0,
                i_GSR       = 0,
                i_GTS       = 0,
                i_KEYCLEARB = 1,
                i_PACK      = 0,
                i_USRCCLKO  = cfgm_clk,
                i_USRCCLKTS = 0,
                i_USRDONEO  = 1,
                i_USRDONETS = 1,
                o_CFGMCLK   = cfgm_clk
            )
            platform.add_period_constraint(cfgm_clk, 1e9/65e6)
        else:
            clk25 = platform.request("clk25")

        # PLL.
        self. pll = pll = S7PLL()
        self.comb += pll.reset.eq(self.rst)
        if use_cfgm_clk:
            pll.register_clkin(cfgm_clk, 65e6)
        else:
            pll.register_clkin(clk25, 25e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        if with_white_rabbit:
            pll.create_clkout(self.cd_clk_125m_gtp,  125e6, margin=0)
            pll.create_clkout(self.cd_clk_125m_dmtd, 125e6, margin=0)
            pll.create_clkout(self.cd_clk_10m_ext,   10e6,  margin=0)
            self.comb += self.cd_refclk_eth.clk.eq(self.cd_clk_125m_gtp.clk)
            platform.add_false_path_constraints(
                pll.clkin,
                self.cd_sys.clk,
                self.cd_clk_125m_dmtd.clk,
                self.cd_clk_125m_gtp.clk,
                self.cd_clk_10m_ext.clk,
            )

        if with_pcie:
            pll.create_clkout(self.cd_clk50, 50e6, margin=0)

        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=125e6,
        # PCIe Parameters.
        with_pcie                 = True,
        with_pcie_ptm             = False,

        # White Rabbit Paramters.
        with_white_rabbit         = True,
        with_white_rabbit_fabric  = False,
        with_white_rabbit_ext_ram = False,
    ):
        # Platform ---------------------------------------------------------------------------------
        platform = Platform(variant="xc7a50t")

        self.file_basedir = os.path.abspath(os.path.dirname(__file__))

        # Clocking ---------------------------------------------------------------------------------

        # General.
        self.crg = _CRG(platform,
            sys_clk_freq      = sys_clk_freq,
            with_white_rabbit = with_white_rabbit,
            with_pcie         = with_pcie,
        )

        # Shared QPLL.
        self.qpll = SharedQPLL(platform,
            with_pcie = with_pcie,
            with_eth  = with_white_rabbit,
        )

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform,
            clk_freq      = sys_clk_freq,
            ident         = "LiteX-WR-NIC on SPEC-A7.",
            ident_version = True,
        )

        # JTAGBone ---------------------------------------------------------------------------------
        self.add_jtagbone()

        # Leds -------------------------------------------------------------------------------------
        if not with_white_rabbit:
            self.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq,
            )

        # PCIe -------------------------------------------------------------------------------------
        if with_pcie:
            self.comb += platform.request("mgt_refclk_125m_oe").eq(1)
            self.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1"),
                data_width  = 64,
                bar0_size   = 0x20000,
                with_ptm    = True,
                refclk_freq = 125e6,
            )
            self.comb += ClockSignal("refclk_pcie").eq(self.pcie_phy.pcie_refclk)
            self.add_pcie(phy=self.pcie_phy,
                ndmas                = 1,
                address_width        = 64,
                with_ptm             = with_pcie_ptm,
                max_pending_requests = 4,
            )
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

        # PCIe PTM ---------------------------------------------------------------------------------
        if with_pcie_ptm:
            assert with_pcie

            # PCIe PTM Sniffer ---------------------------------------------------------------------

            # Since Xilinx PHY does not allow redirecting PTM TLP Messages to the AXI inferface, we have
            # to sniff the GTPE2 -> PCIE2 RX Data to re-generate PTM TLP Messages.

            # Sniffer Signals.
            # ----------------
            sniffer_rst_n   = Signal()
            sniffer_clk     = Signal()
            sniffer_rx_data = Signal(16)
            sniffer_rx_ctl  = Signal(2)

            # Sniffer Tap.
            # ------------
            rx_data = Signal(16)
            rx_ctl  = Signal(2)
            self.sync.pclk += rx_data.eq(rx_data + 1)
            self.sync.pclk += rx_ctl.eq(rx_ctl + 1)
            self.specials += Instance("sniffer_tap",
                i_rst_n_in    = 1,
                i_clk_in     = ClockSignal("pclk"),
                i_rx_data_in = rx_data, # /!\ Fake, will be re-connected post-synthesis /!\.
                i_rx_ctl_in  = rx_ctl,  # /!\ Fake, will be re-connected post-synthesis /!\.

                o_rst_n_out   = sniffer_rst_n,
                o_clk_out     = sniffer_clk,
                o_rx_data_out = sniffer_rx_data,
                o_rx_ctl_out  = sniffer_rx_ctl,
            )

            # Sniffer.
            # --------
            self.pcie_ptm_sniffer = PCIePTMSniffer(
                rx_rst_n = sniffer_rst_n,
                rx_clk   = sniffer_clk,
                rx_data  = sniffer_rx_data,
                rx_ctrl  = sniffer_rx_ctl,
            )
            self.pcie_ptm_sniffer.add_sources(platform)

            # Sniffer Post-Synthesis connections.
            # -----------------------------------
            pcie_ptm_sniffer_connections = []
            for n in range(2):
                pcie_ptm_sniffer_connections.append((
                    f"pcie_s7/inst/inst/gt_top_i/gt_rx_data_k_wire_filter[{n}]", # Src.
                    f"sniffer_tap/rx_ctl_in[{n}]",                               # Dst.
                ))
            for n in range(16):
                pcie_ptm_sniffer_connections.append((
                    f"pcie_s7/inst/inst/gt_top_i/gt_rx_data_wire_filter[{n}]", # Src.
                    f"sniffer_tap/rx_data_in[{n}]",                            # Dst.
                ))
            for _from, _to in pcie_ptm_sniffer_connections:
                platform.toolchain.pre_optimize_commands.append(f"set pin_driver [get_nets -of [get_pins {_to}]]")
                platform.toolchain.pre_optimize_commands.append(f"disconnect_net -net $pin_driver -objects {_to}")
                platform.toolchain.pre_optimize_commands.append(f"connect_net -hier -net {_from} -objects {_to}")

            # Time Generator -----------------------------------------------------------------------

            self.time_generator = TimeGenerator(
                clk_domain = "clk50",
                clk_freq   = 50e6,
            )

            # PTM ----------------------------------------------------------------------------------

            # PTM Capabilities.
            self.ptm_capabilities = PTMCapabilities(
                pcie_endpoint     = self.pcie_endpoint,
                requester_capable = True,
            )

            # PTM Requester.
            self.ptm_requester = PTMRequester(
                pcie_endpoint    = self.pcie_endpoint,
                pcie_ptm_sniffer = self.pcie_ptm_sniffer,
                sys_clk_freq     = sys_clk_freq,
            )
            self.comb += [
                self.ptm_requester.time_clk.eq(ClockSignal("sys")),
                self.ptm_requester.time_rst.eq(ResetSignal("sys")),
                self.ptm_requester.time.eq(self.time_generator.time)
            ]

        # White Rabbit -----------------------------------------------------------------------------

        if with_white_rabbit:
            # Pads.
            # -----
            sfp_pads          = self.platform.request("sfp")
            sfp_i2c_pads      = self.platform.request("sfp_i2c")
            serial_pads       = self.platform.request("serial")

            # Signals.
            # --------
            led_fake_pps = Signal()
            led_pps      = Signal()
            led_link     = Signal()
            led_act      = Signal()
            self.comb += [
                self.platform.request("user_led", 0).eq(~led_link),
                self.platform.request("user_led", 1).eq(~led_act),
                self.platform.request("user_led", 2).eq(~led_pps),
                self.platform.request("user_led", 3).eq(~led_fake_pps),
            ]

            # Clks.
            # -----
            self.cd_wr = ClockDomain("wr")

            # PPS Timer.
            # ----------
            self.pps_timer = pps_timer = ClockDomainsRenamer("clk_10m_ext")(WaitTimer(10e6/2))
            self.comb += pps_timer.wait.eq(~pps_timer.done)
            self.sync.clk_10m_ext += If(pps_timer.done, led_fake_pps.eq(~led_fake_pps))

            # White Rabbit Fabric Interface.
            # ------------------------------
            wrf_src = wishbone.Interface(data_width=16, address_width=2, adressing="byte")
            wrf_snk = wishbone.Interface(data_width=16, address_width=2, adressing="byte")

            self.wrf_stream2wb = wrf_stream2wb = Stream2Wishbone(  cd_to="wr")
            self.wrf_wb2stream = wrf_wb2stream = Wishbone2Stream(cd_from="wr")

            # White Rabbit Slave Interface.
            # -----------------------------
            wb_slave = wishbone.Interface(data_width=32, address_width=32, adressing="byte")
            self.bus.add_slave(name="wr", slave=wb_slave, region=SoCRegion(
                 origin = 0x2000_0000,
                 size   = 0x0100_0000,
             ))

            # White Rabbit Core Instance.
            # ---------------------------
            cpu_firmware = os.path.join(self.file_basedir, "firmware/litex_wr_nic_wrc.bram")
            self.specials += Instance("xwrc_board_artix7_wrapper",
                # Parameters.
                p_g_dpram_initf       = cpu_firmware,

                # Clocks/resets.
                i_areset_n_i          = ~ResetSignal("sys"),
                i_clk_125m_dmtd_i     = ClockSignal("clk_125m_dmtd"),
                i_clk_125m_gtp_i      = ClockSignal("clk_125m_gtp"),
                i_clk_10m_ext_i       = ClockSignal("clk_10m_ext"),

                o_clk_ref_locked_o    = Open(),
                o_dbg_rdy_o           = Open(),
                o_ext_ref_rst_o       = Open(),
                o_clk_ref_62m5_o      = Open(),
                o_clk_62m5_sys_o      = ClockSignal("wr"),
                o_ready_for_reset_o   = Open(),

                # DAC RefClk Interface.
                o_dac_refclk_cs_n_o   = Open(),
                o_dac_refclk_sclk_o   = Open(),
                o_dac_refclk_din_o    = Open(),

                # DAC DMTD Interface.
                o_dac_dmtd_cs_n_o     = Open(),
                o_dac_dmtd_sclk_o     = Open(),
                o_dac_dmtd_din_o      = Open(),

                # SFP Interface.
                o_sfp_txp_o           = sfp_pads.txp,
                o_sfp_txn_o           = sfp_pads.txn,
                i_sfp_rxp_i           = sfp_pads.rxp,
                i_sfp_rxn_i           = sfp_pads.rxn,
                i_sfp_det_i           = 0b1,
                io_sfp_sda            = sfp_i2c_pads.sda,
                io_sfp_scl            = sfp_i2c_pads.scl,
                i_sfp_tx_fault_i      = 0b0,
                i_sfp_tx_los_i        = 0b0,

                # One-Wire Interface.
                i_onewire_i           = 0,
                o_onewire_oen_o       = Open(),

                # UART Interface.
                i_uart_rxd_i          = serial_pads.rx,
                o_uart_txd_o          = serial_pads.tx,

                # SPI Flash Interface.
                o_spi_sclk_o          = Open(),
                o_spi_ncs_o           = Open(),
                o_spi_mosi_o          = Open(),
                i_spi_miso_i          = 0,

                # PPS / Leds.
                i_pps_ext_i           = 0,
                o_pps_p_o             = Open(),
                o_pps_led_o           = led_pps,
                o_led_link_o          = led_link,
                o_led_act_o           = led_act,

                # QPLL Interface (for GTPE2_Common Sharing).
                o_gt0_ext_qpll_reset  = self.qpll.get_channel("eth").reset,
                i_gt0_ext_qpll_clk    = self.qpll.get_channel("eth").clk,
                i_gt0_ext_qpll_refclk = self.qpll.get_channel("eth").refclk,
                i_gt0_ext_qpll_lock   = self.qpll.get_channel("eth").lock,

                # Wishbone Slave Interface (MMAP).
                i_wb_slave_cyc        = wb_slave.cyc,
                i_wb_slave_stb        = wb_slave.stb,
                i_wb_slave_we         = wb_slave.we,
                i_wb_slave_adr        = (wb_slave.adr & 0x0fff_ffff),
                i_wb_slave_sel        = wb_slave.sel,
                i_wb_slave_dat_i      = wb_slave.dat_w,
                o_wb_slave_dat_o      = wb_slave.dat_r,
                o_wb_slave_ack        = wb_slave.ack,
                o_wb_slave_err        = wb_slave.err,
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
                i_wrf_src_stall       = 0, # CHECKME.
                i_wrf_src_err         = wrf_wb2stream.bus.err,
                i_wrf_src_rty         = 0, # CHECKME.

                # Wishbone Fabric Sink Interface.
                i_wrf_snk_adr         = wrf_stream2wb.bus.adr,
                i_wrf_snk_dat         = wrf_stream2wb.bus.dat_w,
                i_wrf_snk_cyc         = wrf_stream2wb.bus.cyc,
                i_wrf_snk_stb         = wrf_stream2wb.bus.stb,
                i_wrf_snk_we          = wrf_stream2wb.bus.we,
                i_wrf_snk_sel         = wrf_stream2wb.bus.sel,

                o_wrf_snk_ack         = wrf_stream2wb.bus.ack,
                o_wrf_snk_stall       = Open(), # CHECKME.
                o_wrf_snk_err         = wrf_stream2wb.bus.err,
                o_wrf_snk_rty         = Open(), # CHECKME.
            )
            self.add_sources()
            self.comb += self.wrf_wb2stream.source.ready.eq(1)

            if with_white_rabbit_fabric:
                # UDP Gen --------------------------------------------------------------------------
                self.udp_gen = UDPPacketGenerator()
                self.comb += self.udp_gen.source.connect(self.wrf_stream2wb.sink)

                # UDP Timer (1s) -------------------------------------------------------------------
                self.udp_timer = udp_timer = WaitTimer(int(125e6))
                self.comb += udp_timer.wait.eq(~udp_timer.done)
                self.comb += self.udp_gen.send.eq(udp_timer.done)

                # UDP/IP Etherbone -----------------------------------------------------------------

                class LiteEthPHYWRGMII(LiteXModule):
                    dw = 8
                    def __init__(self):
                        self.sink   = wrf_stream2wb.sink
                        self.source = wrf_wb2stream.source

                self.ethphy = LiteEthPHYWRGMII()
                self.add_etherbone(phy=self.ethphy, with_timing_constraints=False)

                # Analyzer -------------------------------------------------------------------------
                analyzer_signals = [
                    #wrf_stream2wb.bus,
                    wrf_wb2stream.bus,
                    wrf_wb2stream.fsm,
                    wrf_wb2stream.source,
                    #wrf_stream2wb.sink,
                ]
                self.analyzer = LiteScopeAnalyzer(analyzer_signals,
                    depth        = 256,
                    clock_domain = "sys",
                    samplerate   = int(62.5e6),
                    register     = True,
                    csr_csv      = "analyzer.csv"
                )

    def add_sources(self):
        if not os.path.exists("wr-cores"):
            wr_core_init()
        for file in wr_core_files:
            self.platform.add_source(file)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX-WR-NIC on SPEC-A7.")

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
