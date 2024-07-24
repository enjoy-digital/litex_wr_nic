#!/usr/bin/env python3

# Copyright (C) 2024 Enjoy-Digital.

# ./acorn.py --csr-csv=csr.csv --build --load
# litex_server --jtag --jtag-config=openocd_xc7_ft2232.cfg

# socat - UDP-DATAGRAM:255.255.255.255:24000,broadcast
# enter something
# litescope_cli -r main_basesoc_basesoc_wrf_src_stb

import argparse
import sys
import glob

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex_boards.platforms import sqrl_acorn

from litex.build.generic_platform import *
from litex.build.io               import DifferentialInput
from litex.build.xilinx           import Xilinx7SeriesPlatform
from litex.build.openfpgaloader   import OpenFPGALoader

from litepcie.frontend.ptm  import PCIePTMSniffer
from litepcie.frontend.ptm  import PTMCapabilities, PTMRequester
from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.software      import generate_litepcie_software_headers

from litex.soc.interconnect         import wishbone
from litex.soc.integration.soc      import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.interconnect import stream

from liteeth.phy.a7_gtp import QPLLSettings, QPLL

from litex.soc.interconnect.csr import *

from litex.soc.cores.clock import *

from litescope import LiteScopeAnalyzer

from gateware.time import TimeGenerator

from litescope import LiteScopeAnalyzer

from gateware import list_files

from gateware.udp import UDPPacketGenerator
from gateware.wrf_stream2wb import Stream2Wishbone

# Platform -----------------------------------------------------------------------------------------

class Platform(sqrl_acorn.Platform):
    def create_programmer(self, name="openfpgaloader"):
        return OpenFPGALoader("litex-acorn-baseboard-mini")

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_wr=True, with_pcie=False):
        self.rst              = Signal()
        self.cd_sys           = ClockDomain()
        if with_wr:
            self.cd_clk_125m_dmtd = ClockDomain()
            self.cd_clk_125m_gtp  = ClockDomain()
            self.cd_clk_10m_ext   = ClockDomain()
        if with_pcie:
            self.cd_clk50 = ClockDomain()

        # # #

        # Clk/Rst.
        clk200    = platform.request("clk200")
        clk200_se = Signal()
        self.specials += DifferentialInput(clk200.p, clk200.n, clk200_se)

        # PLL.
        self. pll = pll = S7PLL()
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200_se, 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        if with_wr:
            pll.create_clkout(self.cd_clk_125m_gtp,  125e6, margin=0)
            pll.create_clkout(self.cd_clk_125m_dmtd, 125e6, margin=0)
            pll.create_clkout(self.cd_clk_10m_ext,   10e6,  margin=0)

            platform.add_false_path_constraints(self.cd_clk_125m_dmtd.clk, pll.clkin)
            platform.add_false_path_constraints(self.cd_clk_125m_gtp.clk,  pll.clkin)
            platform.add_false_path_constraints(self.cd_clk_10m_ext.clk,   pll.clkin)

        if with_pcie:
            pll.create_clkout(self.cd_clk50,  50e6, margin=0)

        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=125e6, with_wr=True, with_pcie=True):
        # Platform ---------------------------------------------------------------------------------
        platform = Platform()
        platform.add_extension(sqrl_acorn._litex_acorn_baseboard_mini_io, prepend=True)

        self.file_basedir     = os.path.abspath(os.path.dirname(__file__))
        self.wr_cores_basedir = os.path.join(self.file_basedir, "wr-cores")

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq, with_wr, with_pcie)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform,
            clk_freq      = sys_clk_freq,
            ident         = "PCIe NIC on LiteX Acorn BaseBoard Mini with WR/PTM support.",
            with_jtagbone = True
        )

        # PCIe -------------------------------------------------------------------------------------
        if with_pcie:
            self.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1"),
                data_width = 64,
                bar0_size  = 0x20000,
                with_ptm   = True,
            )
            self.add_pcie(phy=self.pcie_phy,
                ndmas         = 1,
                address_width = 64,
                with_ptm      = True,
            )
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

            # Time ---------------------------------------------------------------------------------

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


        # QPLL -------------------------------------------------------------------------------------

        # PCIe QPLL Settings.
        qpll_pcie_settings = QPLLSettings(
            refclksel  = 0b001,
            fbdiv      = 5,
            fbdiv_45   = 5,
            refclk_div = 1,
        )
        # White Rabbit QPLL Settings.
        qpll_wr_settings = QPLLSettings(
            refclksel  = 0b111,
            fbdiv      = 4,
            fbdiv_45   = 5,
            refclk_div = 1,
        )
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

        # Shared QPLL.
        self.qpll = qpll = QPLL(
            gtrefclk0     = 0 if not with_pcie else self.pcie_phy.pcie_refclk,
            qpllsettings0 = qpll_pcie_settings,
            gtgrefclk1    = self.crg.cd_clk_125m_gtp.clk,
            qpllsettings1 = qpll_wr_settings,
        )
        if with_pcie:
            self.pcie_phy.use_external_qpll(qpll_channel=qpll.channels[0])
        else:
            self.comb += self.qpll.channels[0].reset.eq(0)


        # White Rabbit -----------------------------------------------------------------------------
        if with_wr:
            self._add_white_rabbit_core(cpu_firmware=os.path.join(self.file_basedir, "firmware/speca7_wrc.bram"))

    def _add_white_rabbit_core(self, cpu_firmware):
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
        self.cd_wr      = ClockDomain("wr")
        self.cd_clk62m5 = ClockDomain("clk62m5")
        self.comb += [
            self.cd_clk62m5.rst.eq(self.crg.cd_clk_10m_ext.rst),
        ]

        # PPS Timer.
        # ----------
        self.pps_timer = pps_timer = ClockDomainsRenamer("clk_10m_ext")(WaitTimer(10e6/2))
        self.comb += pps_timer.wait.eq(~pps_timer.done)
        self.sync.clk_10m_ext += If(pps_timer.done, led_fake_pps.eq(~led_fake_pps))

        # White Rabbit Fabric Interface.
        # ------------------------------
        wrf_src = wishbone.Interface(data_width=16, address_width=3, adressing="byte")
        wrf_snk = wishbone.Interface(data_width=16, address_width=3, adressing="byte")
        wrf_snk_stall = Signal()

        # White Rabbit Slave Interface.
        # -----------------------------
        wb_slave = wishbone.Interface(data_width=32, address_width=32, adressing="byte")
        self.bus.add_slave(name="wr", slave=wb_slave, region=SoCRegion(
             origin = 0x2000_0000,
             size   = 0x0100_0000,
         ))

        self.wrf_stream2wb = wrf_stream2wb = Stream2Wishbone(cd_to="wr")

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
            o_clk_ref_62m5_o      = ClockSignal("clk62m5"),
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
            o_gt0_ext_qpll_reset  = self.qpll.channels[1].reset,
            i_gt0_ext_qpll_clk    = self.qpll.channels[1].clk,
            i_gt0_ext_qpll_refclk = self.qpll.channels[1].refclk,
            i_gt0_ext_qpll_lock   = self.qpll.channels[1].lock,

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
            o_wrf_src_adr         = wrf_src.adr,
            o_wrf_src_dat         = wrf_src.dat_w,
            o_wrf_src_cyc         = wrf_src.cyc,
            o_wrf_src_stb         = wrf_src.stb,
            o_wrf_src_we          = wrf_src.we,
            o_wrf_src_sel         = wrf_src.sel,

            i_wrf_src_ack         = wrf_src.ack,
            i_wrf_src_stall       = 0,
            i_wrf_src_err         = wrf_src.err,
            i_wrf_src_rty         = 0,

            # Wishbone Fabric Sink Interface.
            i_wrf_snk_adr         = self.wrf_stream2wb.bus.adr,
            i_wrf_snk_dat         = self.wrf_stream2wb.bus.dat_w,
            i_wrf_snk_cyc         = self.wrf_stream2wb.bus.cyc,
            i_wrf_snk_stb         = self.wrf_stream2wb.bus.stb,
            i_wrf_snk_we          = self.wrf_stream2wb.bus.we,
            i_wrf_snk_sel         = self.wrf_stream2wb.bus.sel,

            o_wrf_snk_ack         = self.wrf_stream2wb.bus.ack,
            o_wrf_snk_stall       = wrf_snk_stall, # FIXME.
            o_wrf_snk_err         = self.wrf_stream2wb.bus.err,
            o_wrf_snk_rty         = Open(),
        )

        # UDP Gen ----------------------------------------------------------------------------------
        self.udp_gen = UDPPacketGenerator()
        self.comb += self.udp_gen.source.connect(self.wrf_stream2wb.sink)

        # UDP Timer (1s) ---------------------------------------------------------------------------
        self.udp_timer = udp_timer = WaitTimer(int(125e6))
        self.comb += udp_timer.wait.eq(~udp_timer.done)
        self.comb += self.udp_gen.send.eq(udp_timer.done)

        # Analyzer ---------------------------------------------------------------------------------
        analyzer_signals = [
            wrf_src,
            #wrf_snk,
        ]
        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 2048,
            clock_domain = "wr",
            samplerate   = int(62.5e6),
            csr_csv      = "analyzer.csv"
        )
        self.add_sources()


    def add_sources(self):
        # fill converter with all path / files required
        custom_files = [
            "gateware/xwrc_platform_vivado.vhd",
            "gateware/xwrc_board_artix7.vhd",
            "gateware/xwrc_board_artix7_wrapper.vhd",
            "gateware/wr_phy/whiterabbit_gtpe2_channel_wrapper.vhd",
            "gateware/wr_phy/whiterabbit_gtpe2_channel_wrapper_gt.vhd",
            "gateware/wr_phy/wr_gtp_phy_family7.vhd",
        ]

        for cf in custom_files:
            self.platform.add_source(os.path.join(self.file_basedir, cf))

        for f in list_files.wr_core_list:
            self.platform.add_source(os.path.join(os.path.abspath(os.path.dirname(__file__)), "wr-cores", f))


# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=Platform, description="Acorn WR.")
    parser.add_target_argument("--flash", action="store_true", help="Flash bitstream to SPI Flash.")
    args = parser.parse_args()

    soc = BaseSoC()

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)
        software_dir = os.path.join(os.path.abspath(os.path.dirname(__file__)), "software")
        generate_litepcie_software_headers(soc, os.path.join(software_dir, "kernel"))

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    if args.flash:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="flash"))

if __name__ == "__main__":
    main()
