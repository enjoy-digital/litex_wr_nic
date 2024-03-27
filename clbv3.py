#!/usr/bin/env python3

# Copyright (C) 2024 Enjoy-Digital.

import sys
import argparse

from litex.gen import *

from litex.build.generic_platform import *
from litex.build.vhd2v_converter import *
from litex.build.xilinx import Xilinx7SeriesPlatform

from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

import list_files

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk / Rst.
    ("clk_125m_dmtd", 0,
        Subsignal("p", Pins("J19")),
        Subsignal("n", Pins("H19")),
        IOStandard("LVDS_25"),
    ),
    ("clk_125m_gpt", 0,
        Subsignal("p", Pins("F6")),
        Subsignal("n", Pins("E6")),
    ),
    ("reset",   0, Pins("C18"),  IOStandard("LVCMOS33")),
    ("suicide", 0, Pins("AB21"), IOStandard("LVCMOS33")),

    # Leds.
    ("led_act",  0, Pins("T20"), IOStandard("LVCMOS33")),
    ("led_link", 0, Pins("AA20"), IOStandard("LVCMOS33")),

    # SPI to DAC.
    ("dac_dmtd", 0,
        Subsignal("clk",  Pins("G22")),
        Subsignal("cs_n", Pins("D22")),
        Subsignal("mosi", Pins("G21")),
        IOStandard("LVCMOS33"),
    ),
    ("dac_refclk", 0,
        Subsignal("clk",  Pins("D21")),
        Subsignal("cs_n", Pins("B22")),
        Subsignal("mosi", Pins("E21")),
        IOStandard("LVCMOS33"),
    ),

    # SFP.
    ("sfp", 0,
        Subsignal("txp", Pins("B4")),
        Subsignal("txn", Pins("A4")),
        Subsignal("rxp", Pins("B8")),
        Subsignal("rxn", Pins("A8")),
    ),
    ("sfp_mod_def0",     0, Pins("W17"),  IOStandard("LVCMOS33")), # SFP detect
    ("sfp_mod_def1",     0, Pins("AA18"), IOStandard("LVCMOS33")), # SFP scl
    ("sfp_mod_def2",     0, Pins("AB18"), IOStandard("LVCMOS33")), # SFP sda
    ("sfp_rate_select",  0, Pins("AB20"), IOStandard("LVCMOS33")),
    ("sfp_tx_fault",     0, Pins("R19"),  IOStandard("LVCMOS33")),
    ("sfp_tx_disable_n", 0, Pins("P19"),  IOStandard("LVCMOS33")),
    ("sfp_rx_los",       0, Pins("V17"),  IOStandard("LVCMOS33")),

    # Onewire.
    ("onewire", 0, Pins("P16"), IOStandard("LVCMOS33")),

    # Serial.
    ("serial", 0,
        Subsignal("tx",  Pins("W5")),
        Subsignal("rx",  Pins("W6")),
        IOStandard("LVCMOS25")
    ),

    # PPS / Monitor.
    ("pps_mon",     0, Pins("F18"), IOStandard("LVCMOS33")),
    ("ref_clk_mon", 0, Pins("E18"), IOStandard("LVCMOS33")),
    
    # Tests.
    ("test_lemo", 0, Pins("B20"), IOStandard("LVCMOS33")),

    # FMC / DIO.
    ("dio", 0,
        Subsignal("clk_p",   Pins("R4"),                   IOStandard("LVDS_25")),
        Subsignal("clk_n",   Pins("T4"),                   IOStandard("LVDS_25")),
        Subsignal("p_i",     Pins(" R6 AA10 T16 M13 J22"), IOStandard("LVDS_25")),
        Subsignal("n_i",     Pins(" T6 AA11 U16 L13 H22"), IOStandard("LVDS_25")),
        Subsignal("p_o",     Pins("W11  AA9 N22 N20 M18"), IOStandard("LVDS_25")),
        Subsignal("n_o",     Pins("W12 AB10 M22 M20 L18"), IOStandard("LVDS_25")),
        Subsignal("oe_n",    Pins("G17 AB13 W16 M15 V10"), IOStandard("LVCMOS25")),
        Subsignal("term_en", Pins("G18  Y12 W10 V13 V14"), IOStandard("LVCMOS25")),
        Subsignal("led_top", Pins("AB11"),                 IOStandard("LVCMOS25")),
        Subsignal("led_bot", Pins("AB12"),                 IOStandard("LVCMOS25")),
    ),
    ("eeprom", 0,
        Subsignal("scl", Pins("V20"), IOStandard("LVCMOS33")),
        Subsignal("sda", Pins("U20"), IOStandard("LVCMOS33")),
    ),

    # BullsEye connector.
    ("bullseye", 0,
        Subsignal("txts_p",          Pins("H13"), IOStandard("LVDS_25")),
        Subsignal("txts_n",          Pins("G13"), IOStandard("LVDS_25")),
        Subsignal("rxts_p",          Pins("G15"), IOStandard("LVDS_25")),
        Subsignal("rxts_n",          Pins("G16"), IOStandard("LVDS_25")),
        Subsignal("pps_p",           Pins("J15"), IOStandard("LVDS_25")),
        Subsignal("pps_n",           Pins("H15"), IOStandard("LVDS_25")),
        Subsignal("clk_ref_62m5_p",  Pins("H17"), IOStandard("LVDS_25")),
        Subsignal("clk_ref_62m5_n",  Pins("H18"), IOStandard("LVDS_25")),
        Subsignal("clk_dmtd_62m5_p", Pins("K21"), IOStandard("LVDS_25")),
        Subsignal("clk_dmtd_62m5_n", Pins("K22"), IOStandard("LVDS_25")),
    ),
]

class Platform(Xilinx7SeriesPlatform):
    default_clk_name   = "clk_125m_dmtd"
    default_clk_period = 1e9/125e6

    def __init__(self, toolchain="vivado"):
        Xilinx7SeriesPlatform.__init__(self, "xc7a200tfbg484-2", _io, toolchain=toolchain)
        self.toolchain.bitstream_commands = [
            "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]",
            "set_property BITSTREAM.CONFIG.SPI_32BIT_ADDR YES [current_design]"
        ]
        self.toolchain.additional_commands = \
            ["write_cfgmem -force -format bin -interface spix4 -size 16 "
             "-loadbit \"up 0x0 {build_name}.bit\" -file {build_name}.bin"]
        self.add_platform_command("set_property INTERNAL_VREF 0.675 [get_iobanks 34]")

    def do_finalize(self, fragment):
        Xilinx7SeriesPlatform.do_finalize(self, fragment)
        self.add_period_constraint(self.lookup_request("clk_125m_dmtd", loose=True), 1e9/125e6)
        self.add_period_constraint(self.lookup_request("clk_125m_gpt", loose=True), 1e9/125e6)

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()

        # # #

        cfgmclk = Signal()
        self.specials += Instance("STARTUPE2",
            i_CLK       = 0,
            i_GSR       = 0,
            i_GTS       = 0,
            i_KEYCLEARB = 1,
            i_PACK      = 0,
            i_USRCCLKO  = cfgmclk,
            i_USRCCLKTS = 0,
            i_USRDONEO  = 1,
            i_USRDONETS = 1,
            o_CFGMCLK   = cfgmclk
        )
        self.comb += self.cd_sys.clk.eq(cfgmclk)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=125e6):
        platform = Platform()

        clk_125m_dmtd = platform.request("clk_125m_dmtd")

        self.crg = _CRG(platform, sys_clk_freq)

        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq)

        clk_125m_gpt = platform.request("clk_125m_gpt")
        dac_refclk   = platform.request("dac_refclk")
        dac_dmtd     = platform.request("dac_dmtd")
        sfp          = platform.request("sfp")
        serial       = platform.request("serial")
        dio          = platform.request("dio")
        eeprom       = platform.request("eeprom")
        bullseye     = platform.request("bullseye")

        self.platform.add_period_constraint(dio.clk_p, 1e9/10e6)

        self.wr_top_params = dict(
            p_g_dpram_initf = "../../../../bin/wrpc/wrc_phy16_direct_dmtd.bram",
            p_g_simulation = 0,
            i_clk_125m_dmtd_p_i = clk_125m_dmtd.p, # 124.992 MHz PLL reference
            i_clk_125m_dmtd_n_i = clk_125m_dmtd.n,
            i_clk_125m_gtp_n_i  = clk_125m_gpt.n,   # 125.000 MHz GTP reference
            i_clk_125m_gtp_p_i  = clk_125m_gpt.p,
            # SPI interface to DACs.
            #-----------------------
            o_dac_refclk_cs_n_o = dac_refclk.cs_n, 
            o_dac_refclk_sclk_o = dac_refclk.clk, 
            o_dac_refclk_din_o  = dac_refclk.mosi, 
            o_dac_dmtd_cs_n_o   = dac_dmtd.cs_n,
            o_dac_dmtd_sclk_o   = dac_dmtd.clk,
            o_dac_dmtd_din_o    = dac_dmtd.mosi,
            # SFP I/O for transceiver.
            #-------------------------
            o_sfp_txp_o         = sfp.txp,
            o_sfp_txn_o         = sfp.txn,
            i_sfp_rxp_i         = sfp.rxp,
            i_sfp_rxn_i         = sfp.rxn,
            i_sfp_mod_def0_i    = platform.request("sfp_mod_def0"),     # sfp detect
            io_sfp_mod_def1_b   = platform.request("sfp_mod_def1"),     # scl
            io_sfp_mod_def2_b   = platform.request("sfp_mod_def2"),     # sda
            o_sfp_rate_select_o = platform.request("sfp_rate_select"),
            i_sfp_tx_fault_i    = platform.request("sfp_tx_fault"),
            o_sfp_tx_disable_o  = platform.request("sfp_tx_disable_n"),
            i_sfp_los_i         = platform.request("sfp_rx_los"),
            # Onewire interface.
            #-------------------
            io_onewire_b        = platform.request("onewire"),
            # UART.
            #------
            i_uart_rxd_i        = serial.rx,
            o_uart_txd_o        = serial.tx,
            # Miscellanous clbv3 pins.
            #-------------------------
            # Red LED next to the SFP: blinking indicates that packets are being
            # transferred.
            o_led_act_o         = platform.request("led_act"),
            # Green LED next to the SFP: indicates if the link is up.
            o_led_link_o        = platform.request("led_link"),
            # Reset control
            i_reset_i           = platform.request("reset"),
            o_suicide           = platform.request("suicide"),
            # test_lemo outputs PPS
            o_test_lemo         = platform.request("test_lemo"),
            # Monitoring signals output on External Debug Connector J35
            o_pps_mon           = platform.request("pps_mon"),
            o_ref_clk_mon       = platform.request("ref_clk_mon"),
            # Digital I/O FMC Pins
            # used in this design to output WR-aligned 1-PPS (in Slave mode) and input
            # 10MHz & 1-PPS from external reference (in GrandMaster mode).
            #-------------------------------------------------------------------------
            # Clock input from LEMO 5 on the mezzanine front panel. Used as 10MHz
            # external reference input.
            i_dio_clk_p_i       = dio.clk_p,
            i_dio_clk_n_i       = dio.clk_n,
            # Differential inputs, dio_p_i(N) inputs the current state of I/O (N+1) on
            # the mezzanine front panel.
            i_dio_n_i           = dio.n_i,
            i_dio_p_i           = dio.p_i,
            # Differential outputs. When the I/O (N+1) is configured as output (i.e. when
            # dio_oe_n_o(N) = 0), the value of dio_p_o(N) determines the logic state
            # of I/O (N+1) on the front panel of the mezzanine
            o_dio_n_o           = dio.n_o,
            o_dio_p_o           = dio.p_o,
            # Output enable. When dio_oe_n_o(N) is 0, connector (N+1) on the front
            # panel is configured as an output.
            o_dio_oe_n_o        = dio.oe_n,
            # Termination enable. When dio_term_en_o(N) is 1, connector (N+1) on the front
            # panel is 50-ohm terminated
            o_dio_term_en_o     = dio.term_en,
            # Two LEDs on the mezzanine panel. Only Top one is currently used - to
            # blink 1-PPS.
            o_dio_led_top_o     = dio.led_top,
            o_dio_led_bot_o     = dio.led_bot,
            # I2C interface for accessing EEPROM.
            io_eeprom_scl_b     = eeprom.scl,
            io_eeprom_sda_b     = eeprom.sda,
            # Bulls-eye connector outputs
            o_txts_p_o          = bullseye.txts_p,
            o_txts_n_o          = bullseye.txts_n,

            o_rxts_p_o          = bullseye.rxts_p,
            o_rxts_n_o          = bullseye.rxts_n,

            o_pps_p_o           = bullseye.pps_p,
            o_pps_n_o           = bullseye.pps_n,

            o_clk_ref_62m5_p_o  = bullseye.clk_ref_62m5_p,
            o_clk_ref_62m5_n_o  = bullseye.clk_ref_62m5_n,

            o_clk_dmtd_62m5_p_o = bullseye.clk_dmtd_62m5_p,
            o_clk_dmtd_62m5_n_o = bullseye.clk_dmtd_62m5_n,
        )

        #self.vhd2v_converter = VHD2VConverter(self.platform,
        #    top_entity = "clbv3_wr_ref_top",
        #    build_dir  = os.path.abspath(os.path.dirname(__file__)),
        #    force_convert = False,
        #)

        # fill converter with all path / files required
        # board specifics
        board_files = [
            "board/clbv3/wr_clbv3_pkg.vhd"
            "board/clbv3/xwrc_board_clbv3.vhd"
            "board/common/wr_board_pkg.vhd"
            "top/clbv3_ref_design/clbv3_wr_ref_top.bmm",
            "top/clbv3_ref_design/clbv3_wr_ref_top.vhd",
        ]

        wr_cores_deps = [
            "ip_cores/general-cores/modules",
            "ip_cores/urv-core/rtl",
            "modules",
            "platform/xilinx/wr_gtp_phy/family7-gtp",
            "platform/xilinx/wr_gtp_phy/gtp_bitslide.vhd",
            "platform/xilinx/wr_xilinx_pkg.vhd",
            "platform/xilinx/xwrc_platform_vivado.vhd",
        ]

        for f in list_files.wr_core_list:
            platform.add_source(os.path.join(os.path.abspath(os.path.dirname(__file__)), "wr-cores", f))

    def do_finalize(self):
        self.specials += Instance("clbv3_wr_ref_top", **self.wr_top_params)


# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=Platform, description="CLBV3 WR.")
    args = parser.parse_args()

    soc = BaseSoC()
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

if __name__ == "__main__":
    main()
