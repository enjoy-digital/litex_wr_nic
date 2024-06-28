#!/usr/bin/env python3

# Copyright (C) 2024 Enjoy-Digital.

# ./acorn.py --csr-csv=csr.csv --build --load
# litex_server --jtag --jtag-config=openocd_xc7_ft2232.cfg

import argparse
import sys
import glob

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from spec_a7_platform import *

from litex.build.generic_platform import *
from litex.build.io               import DifferentialInput
from litex.build.openfpgaloader   import OpenFPGALoader

from litex.soc.integration.soc      import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from liteeth.phy.a7_gtp import QPLLSettings, QPLL

from litex.soc.interconnect.csr import *

from litex.soc.cores.clock      import *

from litescope import LiteScopeAnalyzer

from gateware import list_files

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst             = Signal()
        self.cd_sys          = ClockDomain()
        self.cd_clk_25m_dmtd = ClockDomain()
        self.cd_clk_10m_ext  = ClockDomain()
        self.cd_clk_62_5     = ClockDomain()

        # # #

        # Clk/Rst.
        clk25   = platform.request("clk_25m_dmtd")
        clk62_5 = platform.request("clk62_5")

        clk62_5_se = Signal()
        self.specials += DifferentialInput(clk62_5.p, clk62_5.n, clk62_5_se)

        # PLL.
        self. pll = pll = S7PLL()
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk25, 25e6)
        pll.create_clkout(self.cd_sys,          sys_clk_freq)
        pll.create_clkout(self.cd_clk_25m_dmtd, 25e6, margin=0)
        pll.create_clkout(self.cd_clk_10m_ext,  10e6, margin=0)

        platform.add_false_path_constraints(self.cd_sys.clk,          pll.clkin)
        platform.add_false_path_constraints(self.cd_clk_25m_dmtd.clk, pll.clkin)
        platform.add_false_path_constraints(self.cd_clk_10m_ext.clk,  pll.clkin)

        self.comb += [
            self.cd_clk_62_5.clk.eq(clk62_5_se),
            self.cd_clk_62_5.rst.eq(self.cd_sys.rst),
        ]
 
# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=125e6):
        platform = Platform()

        platform.add_extension([
            ("debug", 0, Pins("ext_misc:2 ext_misc:3 ext_misc:4 ext_misc:5"))
        ])

        self.file_basedir     = os.path.abspath(os.path.dirname(__file__))
        self.wr_cores_basedir = os.path.join(self.file_basedir, "wr-cores")

        self.crg = _CRG(platform, sys_clk_freq)

        SoCMini.__init__(self, platform,
            clk_freq      = sys_clk_freq,
            with_jtagbone = True
        )

        # GTP Clock.
        gtp_refclk      = Signal()
        gtp_refclk_pads = platform.request("mgtrefclk", 1)

        self.specials += Instance("IBUFDS_GTE2",
            i_CEB = 0,
            i_I   = gtp_refclk_pads.p,
            i_IB  = gtp_refclk_pads.n,
            o_O   = gtp_refclk
        )

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
            gtrefclk0     = 0,
            qpllsettings0 = qpll_pcie_settings,
            gtgrefclk1    = gtp_refclk,
            qpllsettings1 = qpll_wr_settings,
        )

        # SFP.
        self.sfp           = platform.request("sfp", 0)
        self.sfp_i2c       = platform.request("sfp_i2c", 0)
        self.sfp_rs        = platform.request("sfp_rs", 0)
        self.sfp_disable_n = platform.request("sfp_disable_n", 0)

        # Flash.
        self.flash      = platform.request("flash")
        self.flash_cs_n = platform.request("flash_cs_n")
        self.flash_clk  = Signal()

        self.serial     = platform.request("serial")
        self.leds       = platform.request_all("user_led")

        self.led_fake_pps = Signal()
        self.led_pps      = Signal()
        self.led_link     = Signal()
        self.led_act      = Signal()
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

        self.control = CSRStorage(fields=[
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

        self.sfp_tx_los   = platform.request("sfp_lose", 0)
        self.sfp_tx_fault = platform.request("sfp_fault", 0)
        self.sfp_det      = Signal()
        self.wr_rstn      = Signal()

        self.comb += [
            self.sfp_det.eq(self.control.fields.sfp_detect),
            self.wr_rstn.eq(~self.rst_ctrl.fields.reset),
        ]

        # Debug
        self.clk_ref_locked   = Signal()
        self.ext_ref_rst      = Signal()
        self.dbg_rdy          = Signal()
        self.clk_ref_62m5     = Signal()
        self.ready_for_reset  = Signal()
        self.debug_pins       = platform.request("debug")
        # dac validation / analyze
        dac_layout            = [("sclk", 1), ("cs_n", 1), ("din", 1)]
        self.dac_dmtd         = Record(dac_layout)
        self.dac_refclk       = Record(dac_layout)

        self.cnt_62m5         = Signal(4)
        self.cnt_125_gtp      = Signal(4)

        #self.comb += [
        #    self.debug_pins.eq(Cat(self.clk_ref_62m5, self.crg.cd_clk_25m_gtp.clk,
        #        self.crg.cd_clk_10m_ext.clk, self.crg.cd_clk_25m_dmtd.clk)),
        #]

        self.sync.clk_62_5 += self.cnt_62m5.eq(self.cnt_62m5 + 1)
        #self.sync.clk_125m_gtp += self.cnt_125_gtp.eq(self.cnt_125_gtp + 1)


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

        # SPI Flash.
        self.specials += Instance("STARTUPE2",
            i_CLK       = 0,
            i_GSR       = 0,
            i_GTS       = 0,
            i_KEYCLEARB = 0,
            i_PACK      = 0,
            i_USRCCLKO  = self.flash_clk,
            i_USRCCLKTS = 0,
            i_USRDONEO  = 1,
            i_USRDONETS = 1,
        )

        self.add_sources()

    def gen_xwrc_board_acorn(self, bram):

        self.specials += Instance("xwrc_board_artix7",
            p_g_simulation                 = 0,
            #p_g_with_external_clock_input  = 1,
            #p_g_dpram_initf               = f"{self.wr_cores_basedir}/bin/wrpc/wrc_phy16_direct_dmtd.bram",
            p_g_DPRAM_INITF                = bram,
            #p_g_fabric_iface              = "PLAIN",
            o_clk_ref_locked_o    = self.clk_ref_locked,
            o_dbg_rdy_o           = self.dbg_rdy,
            o_ext_ref_rst_o       = self.ext_ref_rst,
            o_clk_ref_62m5_o      = self.clk_ref_62m5,

            o_ready_for_reset_o   = self.ready_for_reset,

            i_areset_n_i          = ~ResetSignal("sys") & self.wr_rstn,
            i_clk_125m_dmtd_i     = ClockSignal("clk_25m_dmtd"),
            #i_clk_125m_gtp_i      = ClockSignal("clk_125m_gtp"),
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
            #o_sfp_rate_select_o   = self.sfp_rs, # FIXME/CHECKME.
            i_sfp_tx_fault_i      = self.sfp_tx_fault,
            #o_sfp_tx_disable_o    = self.sfp_disable_n, # FIXME/CHECKME.
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
	        o_spi_sclk_o          = self.flash_clk,
            o_spi_ncs_o           = self.flash_cs_n,
            o_spi_mosi_o          = self.flash.mosi,
            i_spi_miso_i          = self.flash.miso,

            #abscal_txts_o       => wrc_abscal_txts_out,
            #abscal_rxts_o       => wrc_abscal_rxts_out,

            o_pps_ext_i           = 0,#wrc_pps_in,
            #o_pps_p_o             = wrc_pps_out,
            o_pps_led_o           = self.led_pps,
            o_led_link_o          = self.led_link,
            o_led_act_o           = self.led_act,

            o_GT0_EXT_QPLL_RESET  = self.qpll.channels[1].reset,
            i_GT0_EXT_QPLL_CLK    = self.qpll.channels[1].clk,
            i_GT0_EXT_QPLL_REFCLK = self.qpll.channels[1].refclk,
            i_GT0_EXT_QPLL_LOCK   = self.qpll.channels[1].lock,
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
        ]

        custom_files = [
            "gateware/xwrc_platform_vivado.vhd",
            "gateware/xwrc_board_artix7.vhd",
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
    parser = LiteXArgumentParser(platform=Platform, description="SPECA7 WR.")
    parser.add_target_argument("--flash",        action="store_true",       help="Flash bitstream to SPI Flash.")
    args = parser.parse_args()

    soc = BaseSoC()
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    if args.flash:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="flash"))

if __name__ == "__main__":
    main()
