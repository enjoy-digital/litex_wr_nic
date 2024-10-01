#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from litex.soc.interconnect import wishbone

# Wishbone Clock Crossing --------------------------------------------------------------------------

class WishboneClockCrossing(LiteXModule):
    def __init__(self, platform, wb_from, cd_from, wb_to, cd_to):
        self.specials += Instance("xwb_clock_crossing_wrapper",
            # Parameters
            p_g_size = 16,

            # Slave side (sys_clk domain)
            i_slave_clk_i    = ClockSignal(cd_from),
            i_slave_rst_n_i  = ~ResetSignal(cd_from),
            i_slave_cyc_i    = wb_from.cyc,
            i_slave_stb_i    = wb_from.stb,
            i_slave_adr_i    = wb_from.adr,
            i_slave_sel_i    = wb_from.sel,
            i_slave_we_i     = wb_from.we,
            i_slave_dat_i    = wb_from.dat_w,
            o_slave_ack_o    = wb_from.ack,
            o_slave_err_o    = wb_from.err,
            o_slave_rty_o    = Open(),
            o_slave_stall_o  = Open(),
            o_slave_dat_o    = wb_from.dat_r,

            # Master side (wr_clk domain)
            i_master_clk_i   = ClockSignal(cd_to),
            i_master_rst_n_i = ~ResetSignal(cd_to),
            o_master_cyc_o   = wb_to.cyc,
            o_master_stb_o   = wb_to.stb,
            o_master_adr_o   = wb_to.adr,
            o_master_sel_o   = wb_to.sel,
            o_master_we_o    = wb_to.we,
            o_master_dat_o   = wb_to.dat_w,
            i_master_ack_i   = wb_to.ack,
            i_master_err_i   = wb_to.err,
            i_master_rty_i   = Open(),
            i_master_stall_i = Open(),
            i_master_dat_i   = wb_to.dat_r,

            # Flow control
            o_slave_ready_o  = Open(),
            i_slave_stall_i  = 0,
        )
        platform.add_source("gateware/xwb_clock_crossing_wrapper.vhd")
        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_clock_crossing/xwb_clock_crossing.vhd")
