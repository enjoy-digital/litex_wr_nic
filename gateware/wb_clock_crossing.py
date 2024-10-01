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
