#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.soc.interconnect import stream

from litex.soc.interconnect import wishbone


# Wishbone Clock Crossing --------------------------------------------------------------------------

layout_from_to = [
    ("we",     1),
    ("adr",   32),
    ("sel",    4),
    ("dat_w", 32),
]

layout_to_from = [
    ("err",     1),
    ("dat_r",  32),
]

class WishboneClockCrossing(LiteXModule):
    def __init__(self, platform, wb_from, cd_from, wb_to, cd_to):
        self.cdc_from_to = cdc_from_to = stream.ClockDomainCrossing(
            layout  = layout_from_to,
            cd_from = cd_from,
            cd_to   = cd_to,
        )
        self.cdc_to_from = cdc_to_from = stream.ClockDomainCrossing(
            layout  = layout_to_from,
            cd_from = cd_to,
            cd_to   = cd_from,
        )

        self.fsm_from = fsm_from = ClockDomainsRenamer(cd_from)(FSM(reset_state="IDLE"))
        fsm_from.act("IDLE",
            If(wb_from.cyc & wb_from.stb,
                NextState("SEND")
            )
        )
        fsm_from.act("SEND",
            cdc_from_to.sink.valid.eq(1),
            cdc_from_to.sink.we.eq(wb_from.we),
            cdc_from_to.sink.adr.eq(wb_from.adr),
            cdc_from_to.sink.sel.eq(wb_from.sel),
            cdc_from_to.sink.dat_w.eq(wb_from.dat_w),
            If(cdc_from_to.sink.ready,
                NextState("RECEIVE")
            )
        )
        fsm_from.act("RECEIVE",
            cdc_to_from.source.ready.eq(1),
            If(cdc_to_from.source.valid,
                wb_from.ack.eq(1),
                wb_from.err.eq(cdc_to_from.source.err),
                wb_from.dat_r.eq(cdc_to_from.source.dat_r),
                NextState("IDLE")
            )
        )

        self.fsm_to = fsm_to = ClockDomainsRenamer(cd_to)(FSM(reset_state="IDLE"))
        fsm_to.act("IDLE",
            If(cdc_from_to.source.valid,
                NextState("ACCESS")
            )
        )
        self.timer_to = timer_to = ClockDomainsRenamer(cd_to)(WaitTimer(64))
        fsm_to.act("ACCESS",
            wb_to.cyc.eq(1),
            wb_to.stb.eq(1),
            wb_to.we.eq(cdc_from_to.source.we),
            wb_to.adr.eq(cdc_from_to.source.adr),
            wb_to.sel.eq(cdc_from_to.source.sel),
            wb_to.dat_w.eq(cdc_from_to.source.dat_w),
            timer_to.wait.eq(1),
            If(wb_to.ack | timer_to.done,
                cdc_from_to.source.ready.eq(1),
                cdc_to_from.sink.valid.eq(1),
                cdc_to_from.sink.err.eq(wb_to.err),
                cdc_to_from.sink.dat_r.eq(wb_to.dat_r),
                If(timer_to.done,
                    cdc_to_from.sink.err.eq(1),
                    cdc_to_from.sink.dat_r.eq(0xffffffff)
                ),
                NextState("IDLE")
            )
        )


#        self.specials += Instance("xwb_clock_crossing_wrapper",
#            # Parameters.
#            p_g_size = 16,
#
#            # Slave side.
#            i_slave_clk_i    = ClockSignal(cd_from),
#            i_slave_rst_n_i  = ~ResetSignal(cd_from),
#            i_slave_cyc_i    = wb_from.cyc,
#            i_slave_stb_i    = wb_from.stb,
#            i_slave_adr_i    = wb_from.adr,
#            i_slave_sel_i    = wb_from.sel,
#            i_slave_we_i     = wb_from.we,
#            i_slave_dat_i    = wb_from.dat_w,
#            o_slave_ack_o    = wb_from.ack,
#            o_slave_err_o    = wb_from.err,
#            o_slave_rty_o    = Open(),
#            o_slave_stall_o  = Open(),
#            o_slave_dat_o    = wb_from.dat_r,
#
#            # Master side.
#            i_master_clk_i   = ClockSignal(cd_to),
#            i_master_rst_n_i = ~ResetSignal(cd_to),
#            o_master_cyc_o   = wb_to.cyc,
#            o_master_stb_o   = wb_to.stb,
#            o_master_adr_o   = wb_to.adr,
#            o_master_sel_o   = wb_to.sel,
#            o_master_we_o    = wb_to.we,
#            o_master_dat_o   = wb_to.dat_w,
#            i_master_ack_i   = wb_to.ack,
#            i_master_err_i   = wb_to.err,
#            i_master_rty_i   = Open(),
#            i_master_stall_i = Open(),
#            i_master_dat_i   = wb_to.dat_r,
#
#            # Flow control.
#            o_slave_ready_o  = Open(),
#            i_slave_stall_i  = 0,
#        )
#        platform.add_source("gateware/xwb_clock_crossing_wrapper.vhd")
#        platform.add_source("wr-cores/ip_cores/general-cores/modules/wishbone/wb_clock_crossing/xwb_clock_crossing.vhd")
