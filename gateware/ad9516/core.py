#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# AD9516 PLL ---------------------------------------------------------------------------------------

class AD9516PLL(LiteXModule):
    def __init__(self, platform, pads):
        self._rst  = CSRStorage()
        self._done = CSRStatus()

        # # #

        # PLL Driver Instance.
        self.specials += Instance("wr_pll_ctrl",
            p_g_project_name = "NORMAL",
            p_g_spi_clk_freq =  4,
            i_clk_i          = ClockSignal("sys"),
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
        self.add_sources(platform)

    def add_sources(self, platform):
        # SPI Top Sources.
        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_spi/spi_clgen.v")
        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_spi/spi_shift.v")
        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_spi/spi_top.v")
        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_spi/wb_spi.vhd")
        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_spi/timescale.v")
        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_spi/spi_defines.v")
        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_spi/xwb_spi.vhd")

        # WR PLL Ctrl Sources.
        platform.add_source("gateware/ad9516/wr_pll_ctrl_pkg.vhd")
        platform.add_source("gateware/ad9516/wr_pll_ctrl.vhd")
