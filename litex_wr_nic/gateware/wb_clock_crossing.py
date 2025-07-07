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

# Wishbone Clock Crossing --------------------------------------------------------------------------

# This module is a simple/minimal clock crossing module for Wishbone transactions.

class WishboneClockCrossing(LiteXModule):
    def __init__(self, platform, wb_from, cd_from, wb_to, cd_to):
        # S2M CDC (cd_from -> cd_to).
        # ---------------------------
        self.cdc_s2m = cdc_s2m = stream.ClockDomainCrossing(
            layout  = [
                ("we",     1),
                ("adr",   32),
                ("sel",    4),
                ("dat_w", 32),
            ],
            cd_from = cd_from,
            cd_to   = cd_to,
        )
        # M2S CDC (cd_to -> cd_from).
        # ---------------------------
        self.cdc_m2s = cdc_m2s = stream.ClockDomainCrossing(
            layout  = [
                ("err",     1),
                ("dat_r",  32),
            ],
            cd_from = cd_to,
            cd_to   = cd_from,
        )

        # Cross FSM.
        # ----------
        self.cross_fsm = cross_fsm = ClockDomainsRenamer(cd_from)(FSM(reset_state="IDLE"))
        cross_fsm.act("IDLE",
            If(wb_from.cyc & wb_from.stb,
                NextState("SEND")
            )
        )
        cross_fsm.act("SEND",
            # Send request to CDC.
            cdc_s2m.sink.valid.eq(1),
            cdc_s2m.sink.we.eq(wb_from.we),
            cdc_s2m.sink.adr.eq(wb_from.adr),
            cdc_s2m.sink.sel.eq(wb_from.sel),
            cdc_s2m.sink.dat_w.eq(wb_from.dat_w),
            If(cdc_s2m.sink.ready,
                NextState("RECEIVE")
            )
        )
        cross_fsm.act("RECEIVE",
            # Get response from CDC.
            cdc_m2s.source.ready.eq(1),
            If(cdc_m2s.source.valid,
                wb_from.ack.eq(1),
                wb_from.err.eq(cdc_m2s.source.err),
                wb_from.dat_r.eq(cdc_m2s.source.dat_r),
                NextState("IDLE")
            )
        )

        # Access FSM.
        # -----------
        self.access_fsm   = access_fsm = ClockDomainsRenamer(cd_to)(FSM(reset_state="IDLE"))
        self.access_timer = access_timer = ClockDomainsRenamer(cd_to)(WaitTimer(64))
        access_fsm.act("IDLE",
            If(cdc_s2m.source.valid,
                NextState("ACCESS")
            )
        )
        access_fsm.act("ACCESS",
            access_timer.wait.eq(1),
            # Perform Wishbone access.
            wb_to.cyc.eq(1),
            wb_to.stb.eq(1),
            wb_to.we.eq(cdc_s2m.source.we),
            wb_to.adr.eq(cdc_s2m.source.adr),
            wb_to.sel.eq(cdc_s2m.source.sel),
            wb_to.dat_w.eq(cdc_s2m.source.dat_w),
            If(wb_to.ack | access_timer.done,
                # Send response back through CDC.
                cdc_s2m.source.ready.eq(1),
                cdc_m2s.sink.valid.eq(1),
                cdc_m2s.sink.err.eq(wb_to.err),
                cdc_m2s.sink.dat_r.eq(wb_to.dat_r),
                # Err on Timeout.
                If(access_timer.done,
                    cdc_m2s.sink.err.eq(1),
                    cdc_m2s.sink.dat_r.eq(0xffffffff)
                ),
                NextState("IDLE")
            )
        )
