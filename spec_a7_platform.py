#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litex.build.generic_platform import *
from litex.build.xilinx           import Xilinx7SeriesPlatform
from litex.build.openfpgaloader   import OpenFPGALoader

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Rst.
    ("rst", 0, Pins("K15"), IOStandard("LVCMOS33")), # RESET.

    # Free-Running Clk / 125MHz.
    ("clk125_oe", 0, Pins("F14"), IOStandard("LVCMOS25")), # OE_125M.
    ("clk125",    0,
        Subsignal("p", Pins("E16")), # CLK_125MHZ_P.
        Subsignal("n", Pins("D16")), # CLK_125MHZ_N.
        IOStandard("LVDS_25"),
    ),

    # RefClk (GTP) / 125MHz from 25MHz VCXO + AD9516 (X5).
    ("refclk125", 0,
        Subsignal("p", Pins("B6")), # MGTREFCLK1_P.
        Subsignal("n", Pins("B5")), # MGTREFCLK1_N.
    ),
    ("refclk125_syncout", 0,
        Subsignal("p", Pins("E15")), # FPGA_GCLK_P.
        Subsignal("n", Pins("D15")), # FPGA_GCLK_N.
        IOStandard("LVDS_25"),
    ),

    # DMTD Clk / 62.5MHz from VCXO.
    ("clk62m5_dmtd", 0, Pins("T14"), IOStandard("LVCMOS33")), # CLK_25M_DMTD.

    # ExtClk / From 10MHz input.
    ("clk10m_ext", 0,
        Subsignal("p", Pins("E13")), # EXT_CLK_P.
        Subsignal("n", Pins("D14")), # EXT_CLK_N.
        IOStandard("LVDS_25"),
    ),
    ("clk62m5_ext", 0,
        Subsignal("p", Pins("D13")), # CLK_62_5MHZ_P.
        Subsignal("n", Pins("C13")), # CLK_62_5MHZ_N.
        IOStandard("LVDS_25"),
    ),

    # Revision.
    ("revision", 0, Pins("D10 A14 A13"), IOStandard("LVCMOS25")), # HW_REV0-2.

    # Leds.
    ("user_led", 0, Pins("H14"), IOStandard("LVCMOS25")), # LED0 / /!\ Not working on V1.0 /!\.
    ("user_led", 1, Pins("G14"), IOStandard("LVCMOS25")), # LED1 / /!\ Not working on V1.0 /!\.
    ("user_led", 2, Pins("H17"), IOStandard("LVCMOS25")), # LED2 / /!\ Not working on V1.0 /!\.
    ("user_led", 3, Pins("E18"), IOStandard("LVCMOS25")), # LED3 / /!\ Not working on V1.0 /!\.

    ("frontpanel_led", 0, Pins("B11"), IOStandard("LVCMOS25")), # FP_LED0.
    ("frontpanel_led", 1, Pins("B10"), IOStandard("LVCMOS25")), # FP_LED1.
    ("frontpanel_led", 2, Pins("A10"), IOStandard("LVCMOS25")), # FP_LED2.
    ("frontpanel_led", 3, Pins("A12"), IOStandard("LVCMOS25")), # FP_LED3.

    ("clk10m_out_led",  0, Pins("B11"), IOStandard("LVCMOS25")), # FP_LED0.
    ("pps_out_led",     1, Pins("B10"), IOStandard("LVCMOS25")), # FP_LED1.
    ("tod_out_led",     2, Pins("A10"), IOStandard("LVCMOS25")), # FP_LED2.
    ("act_out_led",     3, Pins("A12"), IOStandard("LVCMOS25")), # FP_LED3.

    # GPIOs.
    ("gpio", 0, Pins("M1"), IOStandard("LVCMOS33")), # GPIO0.
    ("gpio", 1, Pins("V3"), IOStandard("LVCMOS33")), # GPIO1.
    ("gpio", 2, Pins("U2"), IOStandard("LVCMOS33")), # GPIO2.
    ("gpio", 3, Pins("N1"), IOStandard("LVCMOS33")), # GPIO3.
    ("gpio", 4, Pins("P1"), IOStandard("LVCMOS33")), # GPIO4.

    # Serial.
    ("serial", 0,
        Subsignal("tx", Pins("R17")), # TX.
        Subsignal("rx", Pins("R16")), # RX.
        IOStandard("LVCMOS33"),
    ),

    # HyperRAM
    ("hyperram", 0,
        Subsignal("clk",   Pins("B14")), # HRAM_CK.
        Subsignal("rst_n", Pins("G17")), # HRAM_RSTn.
        Subsignal("cs_n",  Pins("D18")), # HRAM_CSn.
        Subsignal("dq",    Pins("C16 B16 F18 E17 F17 C17 A17 B15")), # HRAM_DQ0-15.
        Subsignal("rwds",  Pins("C18")), # HRAM_RWDS.
        IOStandard("LVCMOS25")
    ),

    # SPIFlash.
    ("flash", 0,
        Subsignal("cs_n", Pins("L15")), # QSPI_CS.
        Subsignal("mosi", Pins("K16")), # QSPI_DQ0.
        Subsignal("miso", Pins("L17")), # QSPI_DQ1.
        Subsignal("wp",   Pins("J15")), # QSPI_DQ2.
        Subsignal("hold", Pins("J16")), # QSPI_DQ3.
        IOStandard("LVCMOS33"),
    ),

    # PCIe.
    ("pcie_x1", 0,
        Subsignal("rst_n", Pins("R6"), IOStandard("LVCMOS33"), Misc("PULLUP=TRUE")), # PCIe_RST.
        Subsignal("clk_p", Pins("D6")), # PCIe_REFCLK_P.
        Subsignal("clk_n", Pins("D5")), # PCIe_REFCLK_N.
        Subsignal("rx_p",  Pins("E4")), # PCIe_PER0_P.
        Subsignal("rx_n",  Pins("E3")), # PCIe_PER0_N.
        Subsignal("tx_p",  Pins("H2")), # PCIe_PET0_P.
        Subsignal("tx_n",  Pins("H1")), # PCIe_PET0_N.
    ),
    ("pcie_x2", 0,
        Subsignal("rst_n", Pins("R6"), IOStandard("LVCMOS33"), Misc("PULLUP=TRUE")), # PCIe_RST.
        Subsignal("clk_p", Pins("D6")),    # PCIe_REFCLK_P.
        Subsignal("clk_n", Pins("D5")),    # PCIe_REFCLK_N.
        Subsignal("rx_p",  Pins("E4 A4")), # PCIe_PER0-1_P.
        Subsignal("rx_n",  Pins("E3 A3")), # PCIe_PER0-1_N.
        Subsignal("tx_p",  Pins("H2 F2")), # PCIe_PET0-1_P.
        Subsignal("tx_n",  Pins("H1 F1")), # PCIe_PET0-1_N.
    ),

    # RefClk DAC.
    ("dac_refclk", 0,
        Subsignal("ldac_n", Pins("U12")), # DAC_LDAC.
        Subsignal("sync_n", Pins("V13")), # DAC_SYNC.
        Subsignal("sclk",   Pins("V14")), # DAC_SCLK.
        Subsignal("sdi",    Pins("T13")), # DAC_SDI.
        Subsignal("sdo",    Pins("V12")), # DAC_SDO.
        IOStandard("LVCMOS33"),
    ),

    # DMTD DAC.
    ("dac_dmtd", 0,
        Subsignal("ldac_n", Pins("N17")), # DDMTD_LDAC.
        Subsignal("sync_n", Pins("M17")), # DDMTD_SYNC.
        Subsignal("sclk",   Pins("K17")), # DDMTD_SCLK.
        Subsignal("sdi",    Pins("L18")), # DDMTD_SDI.
        Subsignal("sdo",    Pins("N18")), # DDMTD_SDO.
        IOStandard("LVCMOS33"),
    ),

    # AD9516 RefClk PLL.
    ("pll", 0,
        Subsignal("cs_n",    Pins(" V9")), # PLL_CS.
        Subsignal("refsel",  Pins("U15")), # PLL_REFSEL.
        Subsignal("reset_n", Pins("U11")), # PLL_RESET.
        Subsignal("sck",     Pins("V11")), # PLL_SCLK.
        Subsignal("sdi",     Pins("U10")), # PLL_SDI.
        Subsignal("sync_n",  Pins("P16")), # PLL_SYNC.
        Subsignal("lock",    Pins("U16")), # PLL_LOCK.
        Subsignal("sdo",     Pins(" U9")), # PLL_SDO.
        Subsignal("stat",    Pins("V16")), # PLL_STAT.
        IOStandard("LVCMOS33"),
    ),

    # AD9516 Ext PLL.
    ("ext_pll", 0,
        Subsignal("cs_n",    Pins("R3")), # EXT_PLL_CS.
        Subsignal("refsel",  Pins("L4")), # EXT_PLL_REFSEL.
        Subsignal("reset_n", Pins("V6")), # EXT_PLL_RESET.
        Subsignal("sck",     Pins("T2")), # EXT_PLL_SCLK.
        Subsignal("sdi",     Pins("U4")), # EXT_PLL_SDI.
        Subsignal("sync_n",  Pins("R2")), # EXT_PLL_SYNC.
        Subsignal("lock",    Pins("L3")), # EXT_PLL_LOCK.
        Subsignal("sdo",     Pins("V4")), # EXT_PLL_SDO.
        Subsignal("stat",    Pins("R1")), # EXT_PLL_STAT.
        IOStandard("LVCMOS33"),
    ),

    # Sync-Out.
    ("clk10m_out", 0,
        Subsignal("p", Pins("B9")), # SYNC_DATA0_P.
        Subsignal("n", Pins("A9")), # SYNC_DATA0_N.
        IOStandard("LVDS_25"),
    ),
    ("pps_out", 0,
        Subsignal("p", Pins("D8")), # SYNC_DATA1_P.
        Subsignal("n", Pins("C8")), # SYNC_DATA1_P.
        IOStandard("LVDS_25"),
    ),

    # Fine-Delay.
    ("fine_delay", 0,
        Subsignal("en",    Pins("J18")), # DELAY_EN.
        Subsignal("sclk",  Pins("K18")), # DELAY_SCLK.
        Subsignal("sdin",  Pins("J14")), # DELAY_SDIN.
        Subsignal("sload", Pins("M16")), # DELAY_SLOAD.
        IOStandard("LVCMOS33"),
    ),

    # Temp.
    ("temp_1wire", 0, Pins("U14"), IOStandard("LVCMOS33")), # ONE_WIRE.

    # SFP0.
    ("sfp_disable",   0, Pins("U17"),         IOStandard("LVCMOS33")), # SFP0_DISABLE.
    ("sfp_fault",     0, Pins("V17"),         IOStandard("LVCMOS33")), # SFP0_FAULT.
    ("sfp_led",       0, Pins("G16"),         IOStandard("LVCMOS25")), # SFP0.LED.
    ("sfp_los",       0, Pins("P18"),         IOStandard("LVCMOS33")), # SFP0_LOSE.
    ("sfp_mode",      0, Pins("R18 T18 T17"), IOStandard("LVCMOS33")), # SFP0_MODE0-2.
    ("sfp_rs",        0, Pins("N16"),         IOStandard("LVCMOS33")), # SFP0_RS.
    ("sfp_det",       0, Pins("R18"),         IOStandard("LVCMOS33")), # SFP0_MODE0.
    ("sfp_i2c",       0,
        Subsignal("sda", Pins("T17")), # SFP0_MODE2.
        Subsignal("scl", Pins("T18")), # SFP0_MODE1.
        IOStandard("LVCMOS33"),
        Misc("PULLUP True"),
    ),
    ("sfp", 0,
        Subsignal("txp", Pins("D2")), # SFP0_I_P.
        Subsignal("txn", Pins("D1")), # SFP0_I_N.
        Subsignal("rxp", Pins("C4")), # SFP0_O_P.
        Subsignal("rxn", Pins("C3")), # SFP0_O_N.
    ),
    ("sfp_tx", 0,
        Subsignal("p", Pins("D2")), # SFP0_I_P.
        Subsignal("n", Pins("D1")), # SFP0_I_N.
    ),
    ("sfp_rx", 0,
        Subsignal("p", Pins("C4")), # SFP0_O_P.
        Subsignal("n", Pins("C3")), # SFP0_O_N.
    ),

    # SFP1.
    ("sfp_disable",   1, Pins("M15"),         IOStandard("LVCMOS33")), # SFP1_DISABLE.
    ("sfp_fault",     1, Pins("L14"),         IOStandard("LVCMOS33")), # SFP1_FAULT.
    ("sfp_led",       1, Pins("G15"),         IOStandard("LVCMOS25")), # SFP1.LED.
    ("sfp_los",       1, Pins("P15"),         IOStandard("LVCMOS33")), # SFP1_LOSE.
    ("sfp_mode",      1, Pins("T12 N14 M14"), IOStandard("LVCMOS33")), # SFP1_MODE0-2.
    ("sfp_rs",        1, Pins("R13"),         IOStandard("LVCMOS33")), # SFP1_RS.
    ("sfp_det",       1, Pins("T12"),         IOStandard("LVCMOS33")), # SFP1_MODE0.
    ("sfp_i2c",       1,
        Subsignal("sda", Pins("M14")), # SFP1_MODE2.
        Subsignal("scl", Pins("N14")), # SFP1_MODE1.
        IOStandard("LVCMOS33"),
        Misc("PULLUP True"),
    ),
    ("sfp", 1,
        Subsignal("txp", Pins("B2")), # SFP1_I_P.
        Subsignal("txn", Pins("B1")), # SFP1_I_N.
        Subsignal("rxp", Pins("G4")), # SFP1_O_P.
        Subsignal("rxn", Pins("G3")), # SFP1_O_N.
    ),
    ("sfp_tx", 1,
        Subsignal("p", Pins("B2")), # SFP1_I_P.
        Subsignal("n", Pins("B1")), # SFP1_I_N.
    ),
    ("sfp_rx", 1,
        Subsignal("p", Pins("G4")), # SFP1_O_P.
        Subsignal("n", Pins("G3")), # SFP1_O_N.
    ),

    # RGMII.
    ("rgmii", 0,
        Subsignal("rx_dv",   Pins("U1")),          # RGMII_RX_DV.
        Subsignal("rx_clk",  Pins("R5")),          # RGMII_RX_CLK.
        Subsignal("rx_data", Pins("P6 U6 T4 U5")), # RGMII_RXD0-3.
        Subsignal("tx_data", Pins("M6 V2 P5 T5")), # RGMII_TXD0-3.
        Subsignal("tx_en",   Pins("N6")),          # RGMII_TX_EN.
        Subsignal("tx_clk",  Pins("N3")),          # RGMII_TX_CLK.
        IOStandard("LVCMOS33"),
    ),

    # LMX2572.
    ("lmx2572", 0,
        Subsignal("cs_n", Pins("R3")), # LMX_CS.
        Subsignal("clk",  Pins("K1")), # LMX_SCK.
        Subsignal("mosi", Pins("K5")), # LMX_SDI.
        Subsignal("sync", Pins("K5")), # LMX_SYNC.
        IOStandard("LVCMOS33"),
    ),
]

# Connectors ---------------------------------------------------------------------------------------

_connectors = [
    # EXT_MISC.
    ["ext_misc",
        # I2C SDA SCL
        "M5 M2",
    ],
]
# Platform -----------------------------------------------------------------------------------------

class Platform(Xilinx7SeriesPlatform):
    default_clk_name   = "clk62m5"
    default_clk_period = 1e9/62.5e6

    def __init__(self, variant="xc7a35t", toolchain="vivado"):
        assert variant in ["xc7a35t", "xc7a50t"]
        Xilinx7SeriesPlatform.__init__(self, f"{variant}csg325-2", _io,  _connectors, toolchain=toolchain)

        self.toolchain.bitstream_commands = [
            "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 1 [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGRATE 16 [current_design]",
            "set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]",
            "set_property CFGBVS VCCO [current_design]",
            "set_property CONFIG_VOLTAGE 3.3 [current_design]",
        ]

    def create_programmer(self, name="openocd"):
        return OpenFPGALoader(cable="ft4232", fpga_part="xc7a35tcsg324", freq=20e6)

    def do_finalize(self, fragment):
        Xilinx7SeriesPlatform.do_finalize(self, fragment)
        self.add_period_constraint(self.lookup_request("clk62m5", loose=True), 1e9/62.5e6)
