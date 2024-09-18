#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litex.build.generic_platform import *
from litex.build.xilinx import Xilinx7SeriesPlatform, VivadoProgrammer
from litex.build.openocd import OpenOCD

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk / Rst.
    ("clk25",   0, Pins("T14"), IOStandard("LVCMOS33")), # CLK_25M_DMTD.
    ("clk62p5", 0,
        Subsignal("p", Pins("D13"), IOStandard("DIFF_SSTL15")), # CLK_62_5MHZ_P.
        Subsignal("n", Pins("C13"), IOStandard("DIFF_SSTL15")), # CLK_62_5MHZ_N.
    ),
    ("rst", 0, Pins("K15"), IOStandard("LVCMOS33")), # RESET.

    # MGT RefClk.
    ("mgt_refclk_125m_oe", 0, Pins("F14"), IOStandard("LVCMOS25")),
    ("mgt_refclk_125m", 0,
        Subsignal("p", Pins("D6")),
        Subsignal("n", Pins("D5")),
    ),
    ("mgtrefclk", 1,
        Subsignal("p", Pins("B6")),
        Subsignal("n", Pins("B5")),
    ),

    # Leds.
    ("user_led", 0, Pins("H14"), IOStandard("LVCMOS25")),
    ("user_led", 1, Pins("G14"), IOStandard("LVCMOS25")),
    ("user_led", 2, Pins("H17"), IOStandard("LVCMOS25")),
    ("user_led", 3, Pins("E18"), IOStandard("LVCMOS25")),

    # Serial.
    ("serial", 0,
        Subsignal("tx", Pins("R17")),
        Subsignal("rx", Pins("R16")),
        IOStandard("LVCMOS33"),
    ),

    # PCIe.
    ("pcie_x1", 0,
        Subsignal("rst_n", Pins("R6"), IOStandard("LVCMOS33"), Misc("PULLUP=TRUE")),
        Subsignal("clk_p", Pins("D6")), # 125MHz / Free-Running -> Add/Test LitePCIe support.
        Subsignal("clk_n", Pins("D5")), # 125MHz / Free-Running -> Add/Test LitePCIe support.
        Subsignal("rx_p",  Pins("E4")),
        Subsignal("rx_n",  Pins("E3")),
        Subsignal("tx_p",  Pins("H2")),
        Subsignal("tx_n",  Pins("H1")),
    ),
    ("pcie_x2", 0,
        Subsignal("rst_n", Pins("R6"), IOStandard("LVCMOS33"), Misc("PULLUP=TRUE")),
        Subsignal("clk_p", Pins("D6")), # 125MHz / Free-Running -> Add/Test LitePCIe support.
        Subsignal("clk_n", Pins("D5")), # 125MHz / Free-Running -> Add/Test LitePCIe support.
        Subsignal("rx_p",  Pins("E4 C4")),
        Subsignal("rx_n",  Pins("E3 C3")),
        Subsignal("tx_p",  Pins("H2 D2")),
        Subsignal("tx_n",  Pins("H1 D1")),
    ),

    # VCXO Clk Control.
    ("dac_ptp", 0,
        Subsignal("ldac", Pins("U12")),
        Subsignal("sync", Pins("V13")),
        Subsignal("sclk", Pins("V14")),
        Subsignal("sdi",  Pins("T13")),
        Subsignal("sdo",  Pins("V12")), # Unused
        IOStandard("LVCMOS33"),
    ),

    # SFP0.
    ("sfp_disable_n", 0, Pins("U17"),         IOStandard("LVCMOS33")),
    ("sfp_fault",     0, Pins("V17"),         IOStandard("LVCMOS33")),
    ("sfp_led",       0, Pins("G16"),         IOStandard("LVCMOS25")),
    ("sfp_lose",      0, Pins("P18"),         IOStandard("LVCMOS33")),
    ("sfp_mode",      0, Pins("R18 T18 T17"), IOStandard("LVCMOS33")),
    ("sfp_rs",        0, Pins("N16"),         IOStandard("LVCMOS33")),
    ("sfp_i2c",       0,
        Subsignal("sda", Pins("T17")),
        Subsignal("scl", Pins("T18")),
        IOStandard("LVCMOS33"),
    ),
    ("sfp", 0,
        Subsignal("txp", Pins("F2")),
        Subsignal("txn", Pins("F1")),
        Subsignal("rxp", Pins("A4")),
        Subsignal("rxn", Pins("A3")),
    ),
    ("sfp_tx", 0,
        Subsignal("p", Pins("F2")),
        Subsignal("n", Pins("F1")),
    ),
    ("sfp_rx", 0,
        Subsignal("p", Pins("A4")),
        Subsignal("n", Pins("A3")),
    ),

    # SFP1.
    ("sfp_disable_n", 1, Pins("M15"),         IOStandard("LVCMOS33")),
    ("sfp_fault",     1, Pins("L14"),         IOStandard("LVCMOS33")),
    ("sfp_led",       1, Pins("G15"),         IOStandard("LVCMOS25")),
    ("sfp_lose",      1, Pins("P15"),         IOStandard("LVCMOS33")),
    ("sfp_mode",      1, Pins("T12 N14 M14"), IOStandard("LVCMOS33")),
    ("sfp_rs",        1, Pins("R13"),         IOStandard("LVCMOS33")),
    ("sfp_i2c",       1,
        Subsignal("sda", Pins("M14")),
        Subsignal("scl", Pins("N14")),
        IOStandard("LVCMOS33"),
    ),
    ("sfp", 1,
        Subsignal("txp", Pins("B2")),
        Subsignal("txn", Pins("B1")),
        Subsignal("rxp", Pins("G4")),
        Subsignal("rxn", Pins("G3")),
    ),
    ("sfp_tx", 1,
        Subsignal("p", Pins("B2")),
        Subsignal("n", Pins("B1")),
    ),
    ("sfp_rx", 1,
        Subsignal("p", Pins("G4")),
        Subsignal("n", Pins("G3")),
    ),

    # DELAY 0/1.
    ("delay", 0,
        Subsignal("en",    Pins("J18")),
        Subsignal("sclk",  Pins("K18")),
        Subsignal("sdin",  Pins("J14")),
        Subsignal("sload", Pins("M16")),
        Subsignal("sload", Pins("M16")),
        IOStandard("LVCMOS33"),
    ),

    # SPIFlash.
    ("flash_cs_n", 0, Pins("L15"), IOStandard("LVCMOS33")),
    ("flash", 0,
        Subsignal("mosi", Pins("K16")),
        Subsignal("miso", Pins("L17")),
        Subsignal("wp",   Pins("J15")),
        Subsignal("hold", Pins("J16")),
        IOStandard("LVCMOS33"),
    ),

]

# Connectors ---------------------------------------------------------------------------------------

_connectors = [
    # EXT_MISC.
    ["ext_misc",
        # I2C SDA SCL
        "M5 M2",
        # GPIOs
        "M1 M6 N6 N1 P1 M4 N4",
    ],
]
# Platform -----------------------------------------------------------------------------------------

class Platform(Xilinx7SeriesPlatform):
    default_clk_name   = "clk25"
    default_clk_period = 1e9/25e6

    def __init__(self, variant="xc7a35t", toolchain="vivado"):
        assert variant in ["xc7a35t", "xc7a50t"]
        Xilinx7SeriesPlatform.__init__(self, f"{variant}csg325-2", _io,  _connectors, toolchain=toolchain)

        self.toolchain.bitstream_commands = [
            "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGRATE 16 [current_design]",
            "set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]",
            "set_property CFGBVS VCCO [current_design]",
            "set_property CONFIG_VOLTAGE 3.3 [current_design]",
        ]

        self.toolchain.additional_commands = [
            # Non-Multiboot SPI-Flash bitstream generation.
            "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 {build_name}.bit\" -file {build_name}.bin",

            # Multiboot SPI-Flash Operational bitstream generation.
            "set_property BITSTREAM.CONFIG.TIMER_CFG 0x0001fbd0 [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGFALLBACK Enable [current_design]",
            "write_bitstream -force {build_name}_operational.bit ",
            "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 {build_name}_operational.bit\" -file {build_name}_operational.bin",

            # Multiboot SPI-Flash Fallback bitstream generation.
            "set_property BITSTREAM.CONFIG.NEXT_CONFIG_ADDR 0x00400000 [current_design]",
            "write_bitstream -force {build_name}_fallback.bit ",
            "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 {build_name}_fallback.bit\" -file {build_name}_fallback.bin"
        ]

    def create_programmer(self, name="openocd"):
        if name == "openocd":
            return OpenOCD("openocd_xc7_ft232.cfg", "bscan_spi_xc7a35t.bit")
        elif name == "vivado":
            # TODO: some board versions may have s25fl128s
            return VivadoProgrammer(flash_part='s25fl256sxxxxxx0-spi-x1_x2_x4')

    def do_finalize(self, fragment):
        Xilinx7SeriesPlatform.do_finalize(self, fragment)
        self.add_period_constraint(self.lookup_request("clk25", loose=True), 1e9/25e6)
