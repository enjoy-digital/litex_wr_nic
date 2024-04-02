#!/usr/bin/env python3

# Copyright (C) 2024 Enjoy-Digital.

import argparse
import sys
import glob

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.build.generic_platform import *
from litex.build.io import DifferentialInput, Tristate
from litex.build.vhd2v_converter import *
from litex.build.xilinx import Xilinx7SeriesPlatform
from litex.build.openfpgaloader import OpenFPGALoader

from litex.soc.cores.clock import *
from litex.soc.interconnect.csr import *

from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litex_boards.platforms import sqrl_acorn

import list_files

# IOs ----------------------------------------------------------------------------------------------

_ios = [
    ("serial", 0,
        Subsignal("tx", Pins("G1"),  IOStandard("LVCMOS33")), # CLK_REQ
        Subsignal("rx", Pins("Y13"), IOStandard("LVCMOS18")), # SMB_ALERT_N
    ),
    ("sfp", 0,
        Subsignal("txp", Pins("D5")),
        Subsignal("txn", Pins("C5")),
        Subsignal("rxp", Pins("D11")),
        Subsignal("rxn", Pins("C11")),
    ),
    ("sfp_i2c", 0,
        Subsignal("sda", Pins("Y12")),
        Subsignal("scl", Pins("Y11")),
        IOStandard("LVCMOS18"),
    ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(sqrl_acorn.Platform):
    def create_programmer(self, name="openfpgaloader"):
        return OpenFPGALoader("litex-acorn-baseboard-mini")

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
        self. pll = pll = S7PLL()
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200_se, 200e6)
        pll.create_clkout(self.cd_sys,          sys_clk_freq)
        pll.create_clkout(self.cd_clk_125m_gtp, sys_clk_freq)
        pll.create_clkout(self.cd_clk_10m_ext,  10e6)

        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin)
        platform.add_false_path_constraints(self.cd_clk_125m_dmtd.clk, pll.clkin)
        platform.add_false_path_constraints(self.cd_clk_125m_gtp.clk, pll.clkin)
        platform.add_false_path_constraints(self.cd_clk_10m_ext.clk, pll.clkin)

        self.comb += [
            self.cd_clk_125m_dmtd.clk.eq(self.cd_sys.clk),
            self.cd_clk_125m_dmtd.rst.eq(self.cd_sys.rst),
        ]

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=125e6):
        platform = Platform()
        platform.add_extension(_ios, prepend=True)

        file_basedir          = os.path.abspath(os.path.dirname(__file__))
        self.wr_cores_basedir = os.path.join(file_basedir, "wr-cores")

        self.crg = _CRG(platform, sys_clk_freq)

        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq)

        self.sfp        = platform.request("sfp")
        self.sfp_i2c    = platform.request("sfp_i2c")
        self.serial     = platform.request("serial")
        self.leds       = platform.request_all("user_led")

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


        self.gen_xwrc_board_acorn()

        self.comb += self.leds.eq(Cat(~self.led_link, ~self.led_act, ~self.led_pps, ~self.led_fake_pps))

        self.timer = ClockDomainsRenamer("clk_10m_ext")(WaitTimer(10e6/2))
        self.comb += self.timer.wait.eq(~self.timer.done)
        self.sync.clk_10m_ext += If(self.timer.done,
            self.led_fake_pps.eq(~self.led_fake_pps)
        )


        # fill converter with all path / files required
        # board specifics
        board_files = [
            "board/clbv3/wr_clbv3_pkg.vhd",
            #"board/clbv3/xwrc_board_clbv3.vhd",
            "board/common/wr_board_pkg.vhd",
            "board/common/xwrc_board_common.vhd",
            "top/clbv3_ref_design/clbv3_wr_ref_top.bmm",
            "top/clbv3_ref_design/clbv3_wr_ref_top.vhd",
        ]
        platform.add_source(os.path.join(file_basedir, "gateware/xwrc_platform_vivado.vhd"))
        platform.add_source(os.path.join(file_basedir, "gateware/xwrc_board_acorn.vhd"))
        platform.add_source(os.path.join(file_basedir, "gateware/whiterabbit_gtpe2_channel_wrapper_gt.vhd"))
        
        for bf in board_files:
            platform.add_source(os.path.join(self.wr_cores_basedir, bf))

        for f in list_files.wr_core_list:
            platform.add_source(os.path.join(os.path.abspath(os.path.dirname(__file__)), "wr-cores", f))

    def gen_xwrc_board_acorn(self):

        self.specials += Instance("xwrc_board_acorn",
            p_g_simulation                 = 0,
            p_g_with_external_clock_input  = 1,
            #p_g_dpram_initf               = f"{self.wr_cores_basedir}/bin/wrpc/wrc_phy16_direct_dmtd.bram",
            p_g_DPRAM_INITF                = f"{self.wr_cores_basedir}/bin/wrpc/wrc_phy16.bram",
            #p_g_fabric_iface              = "PLAIN",

            i_areset_n_i          = (~ResetSignal("sys") | self.wr_rstn),
            i_clk_125m_dmtd_i     = ClockSignal("clk_125m_dmtd"),
            i_clk_125m_gtp_i      = ClockSignal("clk_125m_gtp"),
            i_clk_10m_ext_i       = ClockSignal("clk_10m_ext"),
            #clk_sys_62m5_o      => clk_sys_62m5,
            #clk_ref_62m5_o      => clk_ref_62m5,
            #clk_dmtd_62m5_o     => clk_dmtd_62m5,
            #rst_sys_62m5_n_o    => rst_sys_62m5_n,
            #rst_ref_62m5_n_o    => rst_ref_62m5_n,

            #dac_refclk_cs_n_o   => dac_refclk_cs_n_o,
            #dac_refclk_sclk_o   => dac_refclk_sclk_o,
            #dac_refclk_din_o    => dac_refclk_din_o,
            #dac_dmtd_cs_n_o     => dac_dmtd_cs_n_o,
            #dac_dmtd_sclk_o     => dac_dmtd_sclk_o,
            #dac_dmtd_din_o      => dac_dmtd_din_o,

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

            #abscal_txts_o       => wrc_abscal_txts_out,
            #abscal_rxts_o       => wrc_abscal_rxts_out,

            o_pps_ext_i           = 0,#wrc_pps_in,
            #o_pps_p_o             = wrc_pps_out,
            o_pps_led_o           = self.led_pps,
            o_led_link_o          = self.led_link,
            o_led_act_o           = self.led_act
        )


    #    clk_pll_62m5 = Signal()
    #    clk_ref_62m5 = Signal()
    #    clk_dmtd     = Signal()
    #    pll_locked   = Signal()
    #    ext_ref_mul  = Signal()
    #    ext_ref_mul_locked  = Signal()
    #    ext_ref_mul_stopped = Signal()
    #    ext_ref_rst         = Signal()
    #    areset_edge_n_i = Signal() # input here
    #    areset_edge_ppulse = Signal()

    #    self.cmd_xwrc_platform_params = dict(
    #        p_g_fpga_family               = "artix7",
    #        p_g_direct_dmtd               = "TRUE",
    #        p_g_with_external_clock_input = "TRUE",
    #        p_g_use_default_plls          = "TRUE",
    #        p_g_simulation                = 0,
    #        i_areset_n_i                  = ~ResetSignal("sys"),
    #        i_clk_10m_ext_i               = ClockSignal("clk_10m_ext"),
    #        i_clk_125m_dmtd_i             = ClockSignal("clk_125m_dmtd"),
    #        i_clk_125m_gtp_p_i            = ClockSignal("clk_125m_gtp"),
    #        #i_clk_125m_gtp_n_i            = clk_125m_gtp_n_i,
    #        i_sfp_txn_o                   = self.sfp.txn,
    #        i_sfp_txp_o                   = self.sfp.txp,
    #        i_sfp_rxn_i                   = self.sfp.rxn,
    #        i_sfp_rxp_i                   = self.sfp.rxp,
    #        i_sfp_tx_fault_i              = Constant(0),
    #        i_sfp_los_i                   = Constant(0),
    #        o_sfp_tx_disable_o            = Open(),
    #        o_clk_62m5_sys_o              = clk_pll_62m5,
    #        o_clk_125m_ref_o              = clk_ref_62m5,  # Note: This is a 62m5 Clock for 16 bit PHYs!
    #        o_clk_62m5_dmtd_o             = clk_dmtd,
    #        o_pll_locked_o                = pll_locked,
    #        o_clk_10m_ext_o               = Open(),
    #        o_phy16_o                     = Open(),
    #        i_phy16_i                     = phy16_from_wrc,
    #        o_ext_ref_mul_o               = ext_ref_mul,
    #        o_ext_ref_mul_locked_o        = ext_ref_mul_locked,
    #        o_ext_ref_mul_stopped_o       = ext_ref_mul_stopped,
    #        i_ext_ref_rst_i               = ext_ref_rst
    #    )

    #    # Reset logic.
    #    # ------------
    #    # Detect when areset_edge_n_i goes high (end of reset) and use this edge to
    #    # generate rstlogic_arst_n. This is needed to connect optional reset like PCIe
    #    # reset. When baord runs standalone, we need to ignore PCIe reset being
    #    # constantly low.
    #    self.specials += Instance("gc_sync_ffs",
    #        p_g_sync_edge = "positive",
    #        i_clk_i       = clk_pll_62m5,
    #        i_rst_n_i     = Constant(1),
    #        i_data_i      = areset_edge_n_i,
    #        o_ppulse_o    = areset_edge_ppulse,
    #    )

    #    # logic AND of all async reset sources (active low)
    #    rstlogic_arst_n <= pll_locked and areset_n_i and (not areset_edge_ppulse);

    #    # concatenation of all clocks required to have synced resets
    #    rstlogic_clk_in(0) <= clk_pll_62m5;
    #    rstlogic_clk_in(1) <= clk_ref_62m5;

    #    self.specials += Instance("gc_reset",
    #        p_g_clocks    = 2, # 62.5MHz, 125MHz
    #        p_g_logdelay  = 4, # 16 clock cycles
    #        p_g_syncdepth = 3, # length of sync chains
    #        i_free_clk_i  = ClockSignal("clk_125m_dmtd"),
    #        i_locked_i    = rstlogic_arst_n,
    #        i_clks_i      = rstlogic_clk_in,
    #        o_rstn_o      = rstlogic_rst_out
    #    )

    #    # distribution of resets (already synchronized to their clock domains)
    #    rst_62m5_n <= rstlogic_rst_out(0);

    #    rst_sys_62m5_n_o <= rst_62m5_n;
    #    rst_ref_62m5_n_o <= rstlogic_rst_out(1);


    #    # 2x SPI DAC
    #    # ----------
    #    #self.specials += Instance("gc_serial_dac", {
    #    #    p_g_num_data_bits => 16,
    #    #    p_g_num_extra_bits => 8,
    #    #    p_g_num_cs_select => 1,
    #    #    p_g_sclk_polarity => 0)
    #    #    i_clk_i         => clk_pll_62m5,
    #    #    i_rst_n_i       => rst_62m5_n,
    #    #    i_value_i       => dac_dmtd_data,
    #    #    i_cs_sel_i      => "1",
    #    #    i_load_i        => dac_dmtd_load,
    #    #    i_sclk_divsel_i => "001",
    #    #    o_dac_cs_n_o(0) => dac_dmtd_cs_n_o,
    #    #    o_dac_sclk_o    => dac_dmtd_sclk_o,
    #    #    o_dac_sdata_o   => dac_dmtd_din_o
    #    #})

    #    #self.specials += Instance("gc_serial_dac",
    #    #    p_g_num_data_bits  = 16,
    #    #    p_g_num_extra_bits = 8,
    #    #    p_g_num_cs_select  = 1,
    #    #    p_g_sclk_polarity  = 0,
    #    #    clk_i         => clk_pll_62m5,
    #    #    rst_n_i       => rst_62m5_n,
    #    #    value_i       => dac_refclk_data,
    #    #    cs_sel_i      => "1",
    #    #    load_i        => dac_refclk_load,
    #    #    sclk_divsel_i => "001",
    #    #    dac_cs_n_o(0) => dac_refclk_cs_n_o,
    #    #    dac_sclk_o    => dac_refclk_sclk_o,
    #    #    dac_sdata_o   => dac_refclk_din_o
    #    #)

    #    # The WR PTP core with optional fabric interface attached
    #    # -------------------------------------------------------

    #    self.specials += Instance("xwrc_board_common",
    #        p_g_simulation                = 0,
    #        p_g_with_external_clock_input = 1, # TRUE
    #        p_g_board_name                = "LAWR", # LiteX Acorn White-Rabbit
    #        p_g_phys_uart                 = 1, # TRUE,
    #        p_g_virtual_uart              = 1, # TRUE,
    #        p_g_aux_clks                  = 0, # 1 for clbv3 g_aux_clks,
    #        p_g_ep_rxbuf_size             = 1024,
    #        p_g_tx_runt_padding           = 1, # TRUE,
    #        p_g_dpram_initf               = g_dpram_initf,
    #        p_g_dpram_size                = 131072/4,
    #        p_g_interface_mode            = "PIPELINED",
    #        p_g_address_granularity       = "BYTE",
    #        p_g_aux_sdb                   = c_wrc_periph3_sdb,
    #        p_g_softpll_enable_debugger   = 0, # FALSE,
    #        p_g_vuart_fifo_size           = 1024,
    #        p_g_pcs_16bit                 = 1, # TRUE,
    #        p_g_diag_id                   = g_diag_id,
    #        p_g_diag_ver                  = g_diag_ver,
    #        p_g_diag_ro_size              = g_diag_ro_size,
    #        p_g_diag_rw_size              = g_diag_rw_size,
    #        p_g_streamers_op_mode         = g_streamers_op_mode,
    #        p_g_tx_streamer_params        = g_tx_streamer_params,
    #        p_g_rx_streamer_params        = g_rx_streamer_params,
    #        p_g_fabric_iface              = g_fabric_iface,
    #        i_clk_sys_i            = clk_pll_62m5,
    #        i_clk_dmtd_i           = clk_dmtd,
    #        i_clk_ref_i            = clk_ref_62m5,
    #        i_clk_aux_i            = Constant(0),
    #        i_clk_10m_ext_i        = clk_10m_ext,
    #        i_clk_ext_mul_i        = ext_ref_mul,
    #        i_clk_ext_mul_locked_i = ext_ref_mul_locked,
    #        i_clk_ext_stopped_i    = ext_ref_mul_stopped,
    #        o_clk_ext_rst_o        = ext_ref_rst,
    #        i_pps_ext_i            = pps_ext_i,
    #        i_rst_n_i              = rst_62m5_n,
    #        o_dac_hpll_load_p1_o   = Open(), #dac_dmtd_load,
    #        o_dac_hpll_data_o      = Open(), #dac_dmtd_data,
    #        o_dac_dpll_load_p1_o   = Open(), #dac_refclk_load,
    #        o_dac_dpll_data_o      = Open(), #dac_refclk_data,
    #        o_phy16_o              = phy16_from_wrc,
    #        i_phy16_i              = phy16_to_wrc,
    #        o_scl_o                = eeprom_scl_o,
    #        i_scl_i                = eeprom_scl_i,
    #        o_sda_o                = eeprom_sda_o,
    #        i_sda_i                = eeprom_sda_i,
    #        o_sfp_scl_o            = sfp_scl_o,
    #        i_sfp_scl_i            = sfp_scl_i,
    #        o_sfp_sda_o            = sfp_sda_o,
    #        i_sfp_sda_i            = sfp_sda_i,
    #        i_sfp_det_i            = sfp_det_i,
    #        o_spi_sclk_o           = open,
    #        o_spi_ncs_o            = open,
    #        o_spi_mosi_o           = open,
    #        i_spi_miso_i           = '0',
    #        i_uart_rxd_i           = self.serial.rx,
    #        o_uart_txd_o           = self.serial.tx,
    #        o_owr_pwren_o          = open,
    #        o_owr_en_o             = onewire_en,
    #        i_owr_i                = onewire_in,
    #        o_wrf_src_o            = wrf_src_o,
    #        i_wrf_src_i            = wrf_src_i,
    #        o_wrf_snk_o            = wrf_snk_o,
    #        i_wrf_snk_i            = wrf_snk_i,
    #        i_wrs_tx_data_i        = 0, #wrs_tx_data_i,
    #        i_wrs_tx_valid_i       = 0, #wrs_tx_valid_i,
    #        o_wrs_tx_dreq_o        = Open(), #wrs_tx_dreq_o,
    #        i_wrs_tx_last_i        = 0, #wrs_tx_last_i,
    #        i_wrs_tx_flush_i       = 0, #wrs_tx_flush_i,
    #        i_wrs_tx_cfg_i         = 0, #wrs_tx_cfg_i,
    #        o_wrs_rx_first_o       = Open(), #wrs_rx_first_o,
    #        o_wrs_rx_last_o        = Open(), #wrs_rx_last_o,
    #        o_wrs_rx_data_o        = Open(), #wrs_rx_data_o,
    #        o_wrs_rx_valid_o       = Open(), #wrs_rx_valid_o,
    #        i_wrs_rx_dreq_i        = 0, #wrs_rx_dreq_i,
    #        i_wrs_rx_cfg_i         = 0, #wrs_rx_cfg_i,
    #        i_aux_diag_i           = aux_diag_i,
    #        o_aux_diag_o           = aux_diag_o,
    #        o_tm_dac_value_o       = tm_dac_value_o,
    #        o_tm_dac_wr_o          = tm_dac_wr_o,
    #        i_tm_clk_aux_lock_en_i = tm_clk_aux_lock_en_i,
    #        o_tm_clk_aux_locked_o  = tm_clk_aux_locked_o,
    #        o_timestamps_o         = timestamps_o,
    #        i_timestamps_ack_i     = timestamps_ack_i,
    #        o_abscal_txts_o        = abscal_txts_o,
    #        o_abscal_rxts_o        = abscal_rxts_o,
    #        i_fc_tx_pause_req_i    = fc_tx_pause_req_i,
    #        i_fc_tx_pause_delay_i  = fc_tx_pause_delay_i,
    #        o_fc_tx_pause_ready_o  = fc_tx_pause_ready_o,
    #        o_tm_link_up_o         = tm_link_up_o,
    #        o_tm_time_valid_o      = tm_time_valid_o,
    #        o_tm_tai_o             = tm_tai_o,
    #        o_tm_cycles_o          = tm_cycles_o,
    #        o_led_act_o            = led_act_o,
    #        o_led_link_o           = led_link_o,
    #        i_btn1_i               = btn1_i,
    #        i_btn2_i               = btn2_i,
    #        o_pps_p_o              = pps_p_o,
    #        o_pps_led_o            = pps_led_o,
    #        o_link_ok_o            = link_ok_o
    #    )

    #    #onewire_oen_o <= onewire_en(0);
    #    #onewire_in(0) <= onewire_i;
    #    #onewire_in(1) <= '1';


# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=Platform, description="Acorn WR.")
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
