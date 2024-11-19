#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# AD9516 PLL Configurations ------------------------------------------------------------------------

# AD916 MAIN:
# - 25MHz VCXO on REF 1.
# - 1500MHz VCO.
# - Out0: Power-Down.
# - Out1: Power-Down.
# - Out2: 500MHz (LVPECL).
# - Out3: 500MHz (LVPECL).
# - Out4: Power-Down.
# - Out5: Power-Down.
# - Out6: 125MHz (LVDS).
# - Out7: 125MHZ (LVDS).
# - Out8: 125MHz (LVDS).
# - Out9: 125MHz (LVDS).

AD9516_MAIN_CONFIG = [
    (0x0000, 0x99), (0x0001, 0x00), (0x0002, 0x10), (0x0003, 0xC3),
    (0x0004, 0x00), (0x0010, 0x7C), (0x0011, 0x05), (0x0012, 0x00),
    (0x0013, 0x0C), (0x0014, 0x12), (0x0015, 0x00), (0x0016, 0x05),
    (0x0017, 0x88), (0x0018, 0x07), (0x0019, 0x00), (0x001A, 0x00),
    (0x001B, 0x00), (0x001C, 0x02), (0x001D, 0x00), (0x001E, 0x00),
    (0x001F, 0x0E), (0x00A0, 0x01), (0x00A1, 0x00), (0x00A2, 0x00),
    (0x00A3, 0x01), (0x00A4, 0x00), (0x00A5, 0x00), (0x00A6, 0x01),
    (0x00A7, 0x00), (0x00A8, 0x00), (0x00A9, 0x01), (0x00AA, 0x00),
    (0x00AB, 0x00), (0x00F0, 0x0A), (0x00F1, 0x0A), (0x00F2, 0x08),
    (0x00F3, 0x08), (0x00F4, 0x0A), (0x00F5, 0x0A), (0x0140, 0x42),
    (0x0141, 0x42), (0x0142, 0x42), (0x0143, 0x42), (0x0190, 0x00),
    (0x0191, 0x08), (0x0192, 0x00), (0x0193, 0x00), (0x0194, 0x80),
    (0x0195, 0x00), (0x0196, 0x00), (0x0197, 0x80), (0x0198, 0x00),
    (0x0199, 0x11), (0x019A, 0x00), (0x019B, 0x00), (0x019C, 0x20),
    (0x019D, 0x00), (0x019E, 0x11), (0x019F, 0x00), (0x01A0, 0x00),
    (0x01A1, 0x20), (0x01A2, 0x00), (0x01A3, 0x00), (0x01E0, 0x01),
    (0x01E1, 0x02), (0x0230, 0x00), (0x0231, 0x00), (0x0232, 0x01),
]

# AD916 EXT:
# - 10MHz on REF IN (LVDS).
# - 1500MHz VCO.
# - Out6: 62.5MHz (LVDS).
# - OutX: Power-Down.

AD9516_EXT_CONFIG = [
    (0x0000, 0x99), (0x0001, 0x00), (0x0002, 0x10), (0x0003, 0xC3),
    (0x0004, 0x00), (0x0010, 0x7C), (0x0011, 0x04), (0x0012, 0x00),
    (0x0013, 0x08), (0x0014, 0x25), (0x0015, 0x00), (0x0016, 0x05),
    (0x0017, 0x88), (0x0018, 0x07), (0x0019, 0x00), (0x001A, 0x00),
    (0x001B, 0x00), (0x001C, 0x41), (0x001D, 0x00), (0x001E, 0x00),
    (0x001F, 0x0E), (0x00A0, 0x01), (0x00A1, 0x00), (0x00A2, 0x00),
    (0x00A3, 0x01), (0x00A4, 0x00), (0x00A5, 0x00), (0x00A6, 0x01),
    (0x00A7, 0x00), (0x00A8, 0x00), (0x00A9, 0x01), (0x00AA, 0x00),
    (0x00AB, 0x00), (0x00F0, 0x0A), (0x00F1, 0x0A), (0x00F2, 0x0A),
    (0x00F3, 0x0A), (0x00F4, 0x0A), (0x00F5, 0x0A), (0x0140, 0x42),
    (0x0141, 0x43), (0x0142, 0x43), (0x0143, 0x43), (0x0190, 0x00),
    (0x0191, 0x08), (0x0192, 0x00), (0x0193, 0x00), (0x0194, 0x80),
    (0x0195, 0x00), (0x0196, 0x00), (0x0197, 0x80), (0x0198, 0x00),
    (0x0199, 0x55), (0x019A, 0x00), (0x019B, 0x00), (0x019C, 0x20),
    (0x019D, 0x00), (0x019E, 0x11), (0x019F, 0x00), (0x01A0, 0x00),
    (0x01A1, 0x20), (0x01A2, 0x00), (0x01A3, 0x00), (0x01E0, 0x00),
    (0x01E1, 0x02), (0x0230, 0x00), (0x0231, 0x00), (0x0232, 0x01),
]

# AD9516 PLL ---------------------------------------------------------------------------------------

class AD9516PLL(LiteXModule):
    def __init__(self, platform, pads, config, name, clk_domain="sys"):
        self._rst  = CSRStorage()
        self._done = CSRStatus()

        # # #

        # PLL Driver Instance.
        self.specials += Instance(f"wr_pll_ctrl_{name}",
            p_g_project_name = "NORMAL",
            p_g_spi_clk_freq =  4,
            i_clk_i          = ClockSignal(clk_domain),
            i_rst_n_i        = ~self._rst.storage,
            i_pll_lock_i     = pads.lock,
            o_pll_reset_n_o  = pads.reset_n,
            i_pll_status_i   = pads.stat,
            o_pll_refsel_o   = pads.refsel,
            o_pll_sync_n_o   = pads.sync_n,
            o_pll_cs_n_o     = pads.cs_n,
            o_pll_sck_o      = pads.sck,
            o_pll_mosi_o     = pads.sdi,
            i_pll_miso_i     = pads.sdo,
            o_done_o         = self._done.status,
        )
        self.add_sources(platform, config, name)

    def add_sources(self, platform, config, name):
        # Generate VHDL package.
        self.generate_vhdl_package(config, name)

        # SPI Top Sources.
        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_spi/spi_clgen.v")
        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_spi/spi_shift.v")
        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_spi/spi_top.v")
        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_spi/wb_spi.vhd")
        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_spi/timescale.v")
        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_spi/spi_defines.v")
        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_spi/xwb_spi.vhd")

        # WR PLL Ctrl Sources.
        platform.add_source(f"gateware/ad9516/wr_pll_ctrl_{name}_pkg.vhd")
        platform.add_source(f"gateware/ad9516/wr_pll_ctrl_{name}.vhd")

    def generate_vhdl_package(self, config, name):
        """Generate the VHDL package for wr_pll_ctrl."""
        vhdl_template = """\
library ieee;
use ieee.std_logic_1164.all;

package wr_pll_ctrl_{name}_pkg is

  type t_data_array is array(natural range <>) of std_logic_vector(7 downto 0);
  type t_addr_array is array(natural range <>) of std_logic_vector(15 downto 0);

  -- AD9516 settings
  constant c_spi_addr_array : t_addr_array := (
    {addr_array}
  );

  constant c_spi_data_array : t_data_array := (
    {data_array}
  );

end wr_pll_ctrl_{name}_pkg;
"""
        addr_array_str = ",\n    ".join([f"x\"{addr:04X}\"" for addr, _ in config])
        data_array_str = ",\n    ".join([f"x\"{data:02X}\"" for _, data in config])

        vhdl_content = vhdl_template.format(
            addr_array = addr_array_str,
            data_array = data_array_str,
            name       = name,
        )

        output_dir = os.path.join(os.getcwd(), "gateware/ad9516")
        os.makedirs(output_dir, exist_ok=True)
        output_file = os.path.join(output_dir, f"wr_pll_ctrl_{name}_pkg.vhd")

        with open(output_file, "w") as f:
            f.write(vhdl_content)

        print(f"Generated VHDL package: {output_file}")