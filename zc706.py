#!/usr/bin/env python3

# Copyright (C) 2024 Enjoy-Digital.

# ./acorn.py --csr-csv=csr.csv --build --load
# litex_server --jtag --jtag-config=openocd_xc7_ft2232.cfg

import argparse
import sys
import glob

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.build.generic_platform import *
from litex.build.io import DifferentialInput, Tristate
from litex.build.vhd2v_converter import *
from litex.build.xilinx import Xilinx7SeriesPlatform

from litex.soc.cores.clock import *
from litex.soc.interconnect.csr import *

from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litex_boards.platforms import xilinx_zc706

from litescope import LiteScopeAnalyzer

import list_files

# IOs ----------------------------------------------------------------------------------------------

_ios = [
    ("serial", 0,
        # USB-UART PMOD: https://store.digilentinc.com/pmod-usbuart-usb-to-uart-interface/
        Subsignal("tx", Pins("pmod1:1"), IOStandard("LVCMOS25")), # CLK_REQ
        Subsignal("rx", Pins("pmod1:2"), IOStandard("LVCMOS25")), # SMB_ALERT_N
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst              = Signal()
        self.cd_sys           = ClockDomain()
        self.cd_clk_125m_dmtd = ClockDomain()
        self.cd_clk_125m_gtp  = ClockDomain()
        self.cd_clk_10m_ext   = ClockDomain()

        # # #

        # Clk/Rst.
        clk200    = platform.request("clk200")
        clk200_se = Signal()
        self.specials += DifferentialInput(clk200.p, clk200.n, clk200_se)

        # PLL.
        self. pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200_se, 200e6)
        pll.create_clkout(self.cd_sys,           sys_clk_freq)
        pll.create_clkout(self.cd_clk_125m_gtp,  125e6, margin=0)
        pll.create_clkout(self.cd_clk_125m_dmtd, 125e6, margin=0)
        pll.create_clkout(self.cd_clk_10m_ext,   10e6,  margin=0)

        platform.add_false_path_constraints(self.cd_sys.clk,           pll.clkin)
        platform.add_false_path_constraints(self.cd_clk_125m_dmtd.clk, pll.clkin)
        platform.add_false_path_constraints(self.cd_clk_125m_gtp.clk,  pll.clkin)
        platform.add_false_path_constraints(self.cd_clk_10m_ext.clk,   pll.clkin)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=125e6):
        platform = xilinx_zc706.Platform()
        platform.add_extension(_ios, prepend=True)
        print(platform.constraint_manager.available)
        for v in platform.constraint_manager.available:
            print(v[0])

        self.file_basedir     = os.path.abspath(os.path.dirname(__file__))
        self.wr_cores_basedir = os.path.join(self.file_basedir, "wr-cores")

        self.crg = _CRG(platform, sys_clk_freq)

        SoCMini.__init__(self, platform,
            clk_freq      = sys_clk_freq,
            with_jtagbone = True
        )

        self.sfp        = platform.request("sfp")
        self.sfp_i2c    = platform.request("i2c")
        self.serial     = platform.request("serial")
        self.leds       = platform.request_all("user_led")
        #self.flash      = platform.request("flash")
        #self.flash_cs_n = platform.request("flash_cs_n")
        #self.flash_clk  = Signal()

        self.led_fake_pps = Signal()
        self.led_pps      = Signal()
        self.led_link     = Signal()
        self.led_act      = Signal()
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

        self.control = CSRStorage(fields=[
            CSRField("sfp_los", size=1, offset=0, values=[
                ("``0b0``", "Set Low."),
                ("``0b1``", "Set High.")
            ], reset=0),
            CSRField("sfp_fault", size=1, offset=1, values=[
                ("``0b0``", "Set Low."),
                ("``0b1``", "Set High.")
            ], reset=0),
            CSRField("sfp_detect", size=1, offset=2, values=[
                ("``0b0``", "Set Low."),
                ("``0b1``", "Set High.")
            ], reset=1),
        ])

        self.rst_ctrl = CSRStorage(fields=[
            CSRField("reset", size=1, offset=0, values=[
                ("``0b0``", "Normal Mode."),
                ("``0b1``", "Reset Mode.")
            ], reset=0),
        ])

        self.sfp_tx_los   = Signal()
        self.sfp_tx_fault = Signal()
        self.sfp_det      = Signal()
        self.wr_rstn      = Signal()

        self.comb += [
            self.sfp_tx_los.eq(self.control.fields.sfp_los),
            self.sfp_tx_fault.eq(self.control.fields.sfp_fault),
            self.sfp_det.eq(self.control.fields.sfp_detect),
            self.wr_rstn.eq(~self.rst_ctrl.fields.reset),
        ]

        # Debug
        self.clk_ref_locked   = Signal()
        self.ext_ref_rst      = Signal()
        self.dbg_rdy          = Signal()
        self.clk_ref_62m5     = Signal()
        self.ready_for_reset  = Signal()
        # dac validation / analyze
        dac_layout            = [("sclk", 1), ("cs_n", 1), ("din", 1)]
        self.dac_dmtd         = Record(dac_layout)
        self.dac_refclk       = Record(dac_layout)

        self.cd_clk62m5       = ClockDomain()
        self.cnt_62m5         = Signal(4)
        self.cnt_125_gtp      = Signal(4)

        self.comb += [
            self.cd_clk62m5.clk.eq(self.clk_ref_62m5),
            self.cd_clk62m5.rst.eq(self.crg.cd_clk_10m_ext.rst),
        ]

        self.sync.clk62m5 += self.cnt_62m5.eq(self.cnt_62m5 + 1)
        self.sync.clk_125m_gtp += self.cnt_125_gtp.eq(self.cnt_125_gtp + 1)


        self.debug  = debug = Signal(32)

        # GTPE2_CHANNEL.
        GTPE2_CHANNEL_GT0_PLL1RESET_IN    = Signal()
        GTPE2_CHANNEL_GT0_DRP_BUSY_OUT    = Signal()
        GTPE2_CHANNEL_GT0_GTRXRESET_IN    = Signal()
        GTPE2_CHANNEL_GT0_GTTXRESET_IN    = Signal()
        GTPE2_CHANNEL_GT0_RXRESETDONE_OUT = Signal()
        GTPE2_CHANNEL_GT0_TXRESETDONE_OUT = Signal()
        self.comb += [
            GTPE2_CHANNEL_GT0_PLL1RESET_IN.eq(   debug[0]),
            GTPE2_CHANNEL_GT0_DRP_BUSY_OUT.eq(   debug[1]),
            GTPE2_CHANNEL_GT0_GTRXRESET_IN.eq(   debug[2]),
            GTPE2_CHANNEL_GT0_GTTXRESET_IN.eq(   debug[3]),
            GTPE2_CHANNEL_GT0_RXRESETDONE_OUT.eq(debug[4]),
            GTPE2_CHANNEL_GT0_TXRESETDONE_OUT.eq(debug[5]),
        ]

        # GTPE2_COMMON.
        GTPE2_COMMON_GT0_PLL1RESET_IN       = Signal()
        GTPE2_COMMON_GT0_PLL1LOCKDETCLK_IN  = Signal()
        GTPE2_COMMON_GT0_PLL1LOCK_OUT       = Signal()
        GTPE2_COMMON_GT0_PLL1REFCLKLOST_OUT = Signal()
        self.comb += [
            GTPE2_COMMON_GT0_PLL1RESET_IN.eq(      debug[16]),
            GTPE2_COMMON_GT0_PLL1LOCKDETCLK_IN.eq( debug[17]),
            GTPE2_COMMON_GT0_PLL1LOCK_OUT.eq(      debug[18]),
            GTPE2_COMMON_GT0_PLL1REFCLKLOST_OUT.eq(debug[19]),
        ]

        # WR core
        self.gen_xwrc_board_acorn(os.path.join(self.file_basedir, "wrc_acorn.bram"))

        self.comb += self.leds.eq(Cat(~self.led_link, ~self.led_act, ~self.led_pps, ~self.led_fake_pps))

        self.timer = ClockDomainsRenamer("clk_10m_ext")(WaitTimer(10e6/2))
        self.comb += self.timer.wait.eq(~self.timer.done)
        self.sync.clk_10m_ext += If(self.timer.done,
            self.led_fake_pps.eq(~self.led_fake_pps)
        )

        self.add_sources()

        cnt1 = Signal()
        cnt2 = Signal()

        self.comb += [
            cnt1.eq(self.cnt_125_gtp[3]),
            cnt2.eq(self.cnt_62m5[3]),
        ]

        analyzer_signals = []
        if False:
            analyzer_signals += [
                self.crg.cd_clk_125m_dmtd.rst,
                self.ext_ref_rst,
                self.clk_ref_locked,
                self.dbg_rdy,
                self.ready_for_reset,
                cnt1,
                cnt2,
                GTPE2_CHANNEL_GT0_PLL1RESET_IN,
                GTPE2_CHANNEL_GT0_DRP_BUSY_OUT,
                GTPE2_CHANNEL_GT0_GTRXRESET_IN,
                GTPE2_CHANNEL_GT0_GTTXRESET_IN,
                GTPE2_CHANNEL_GT0_RXRESETDONE_OUT,
                GTPE2_CHANNEL_GT0_TXRESETDONE_OUT,
                GTPE2_COMMON_GT0_PLL1RESET_IN,
                GTPE2_COMMON_GT0_PLL1LOCKDETCLK_IN,
                GTPE2_COMMON_GT0_PLL1LOCK_OUT,
                GTPE2_COMMON_GT0_PLL1REFCLKLOST_OUT,
            ]

        if True:
            analyzer_signals += [
                self.dac_refclk,
                self.dac_dmtd,
            ]

        self.analyzer = LiteScopeAnalyzer(analyzer_signals,
            depth        = 256,
            clock_domain = "sys",
            register     = True,
            csr_csv      = "analyzer.csv"
        )

    def gen_xwrc_board_acorn(self, bram):

        self.specials += Instance("xwrc_board_zc706",
            p_g_simulation                 = 0,
            #p_g_with_external_clock_input  = 1,
            p_g_DPRAM_INITF                = bram,
            #p_g_fabric_iface              = "PLAIN",
            o_clk_ref_locked_o    = self.clk_ref_locked,
            o_dbg_rdy_o           = self.dbg_rdy,
            o_ext_ref_rst_o       = self.ext_ref_rst,
            o_clk_ref_62m5_o      = self.clk_ref_62m5,

            o_ready_for_reset_o   = self.ready_for_reset,

            i_areset_n_i          = ~ResetSignal("sys") & self.wr_rstn,
            i_clk_125m_dmtd_i     = ClockSignal("clk_125m_dmtd"),
            i_clk_125m_gtp_i      = ClockSignal("clk_125m_gtp"),
            i_clk_10m_ext_i       = ClockSignal("clk_10m_ext"),
            #clk_sys_62m5_o      => clk_sys_62m5,
            #clk_ref_62m5_o      => clk_ref_62m5,
            #clk_dmtd_62m5_o     => clk_dmtd_62m5,
            #rst_sys_62m5_n_o    => rst_sys_62m5_n,
            #rst_ref_62m5_n_o    => rst_ref_62m5_n,

            o_dac_refclk_cs_n_o   = self.dac_refclk.cs_n,
            o_dac_refclk_sclk_o   = self.dac_refclk.sclk,
            o_dac_refclk_din_o    = self.dac_refclk.din,
            o_dac_dmtd_cs_n_o     = self.dac_dmtd.cs_n,
            o_dac_dmtd_sclk_o     = self.dac_dmtd.sclk,
            o_dac_dmtd_din_o      = self.dac_dmtd.din,

            o_sfp_txp_o           = self.sfp.txp,
            o_sfp_txn_o           = self.sfp.txn,
            i_sfp_rxp_i           = self.sfp.rxp,
            i_sfp_rxn_i           = self.sfp.rxn,
            i_sfp_det_i           = self.sfp_det,
            io_sfp_sda            = self.sfp_i2c.sda,
            io_sfp_scl            = self.sfp_i2c.scl,
            #sfp_rate_select_o   => self.sfp_rate_select_o,
            i_sfp_tx_fault_i      = self.sfp_tx_fault,
            #sfp_tx_disable_o    => self.sfp_tx_disable_o,
            i_sfp_tx_los_i        = self.sfp_tx_los,

            #eeprom_sda_i        => eeprom_sda_in,
            #eeprom_sda_o        => eeprom_sda_out,
            #eeprom_scl_i        => eeprom_scl_in,
            #eeprom_scl_o        => eeprom_scl_out,

            #onewire_i           => onewire_data,
            #onewire_oen_o       => onewire_oe,
            # Uart
            i_uart_rxd_i          = self.serial.rx,
            o_uart_txd_o          = self.serial.tx,

            # SPI Flash
	        o_spi_sclk_o          = Open(),
            o_spi_ncs_o           = Open(),
            o_spi_mosi_o          = Open(),
            i_spi_miso_i          = 1,

            #abscal_txts_o       => wrc_abscal_txts_out,
            #abscal_rxts_o       => wrc_abscal_rxts_out,

            o_pps_ext_i           = 0,#wrc_pps_in,
            #o_pps_p_o             = wrc_pps_out,
            o_pps_led_o           = self.led_pps,
            o_led_link_o          = self.led_link,
            o_led_act_o           = self.led_act,

            o_debug               = self.debug,
        )

    def add_sources(self):
        # fill converter with all path / files required
        # board specifics
        board_files = [
            "board/clbv3/wr_clbv3_pkg.vhd",
            "board/common/wr_board_pkg.vhd",
            "board/common/xwrc_board_common.vhd",
            "top/clbv3_ref_design/clbv3_wr_ref_top.bmm",
            "top/clbv3_ref_design/clbv3_wr_ref_top.vhd",
            "platform/xilinx/wr_gtp_phy/family7-gtx/whiterabbit_gtxe2_channel_wrapper_gt.vhd",
            "platform/xilinx/wr_gtp_phy/family7-gtx/wr_gtx_phy_family7.vhd",
        ]

        custom_files = [
            "gateware/xwrc_platform_vivado.vhd",
            "gateware/xwrc_board_zc706.vhd",
            "gateware/whiterabbit_gtpe2_channel_wrapper.vhd",
            "gateware/whiterabbit_gtpe2_channel_wrapper_gt.vhd",
            "gateware/whiterabbit_gtpe2_channel_wrapper_gtrxreset_seq.vhd",
            "gateware/wr_gtp_phy_family7.vhd",
        ]

        for cf in custom_files:
            self.platform.add_source(os.path.join(self.file_basedir, cf))

        for bf in board_files:
            self.platform.add_source(os.path.join(self.wr_cores_basedir, bf))

        for f in list_files.wr_core_list:
            self.platform.add_source(os.path.join(os.path.abspath(os.path.dirname(__file__)), "wr-cores", f))


# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=xilinx_zc706.Platform, description="ZC706 WR.")
    args = parser.parse_args()

    soc = BaseSoC()
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
