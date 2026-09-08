#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from litex_wr_nic.gateware.wrf import WRFInterface, WRFClockCrossing, WRFCounters

# White Rabbit Fabric Wishbone to Stream -----------------------------------------------------------

class Wishbone2Stream(LiteXModule):
    """Receive WR packets without acknowledging data that cannot be buffered.

    An explicit end token preserves CYC termination while the FIFO is full.
    One data word is retained until the following word or end token is known,
    so final status/error words and odd byte counts reach the correct packet.
    """
    def __init__(self, cd_from="wr", depth=16):
        self.bus    = bus    = WRFInterface()
        self.source = source = stream.Endpoint([("data", 8), ("error", 1)])

        # # #

        layout = [("data", 16), ("adr", 2), ("sel", 2), ("end", 1), ("error", 1)]
        self.cdc = cdc = WRFClockCrossing(layout, cd_from, "sys", depth)
        self.stats = stats = WRFCounters(cdc.input_cd)
        self.input_fsm = ingress = ClockDomainsRenamer(cdc.input_cd)(FSM(reset_state="IDLE"))
        accepted     = Signal()
        frame_bad    = Signal()
        stalled      = Signal()
        stalled_word = Signal(21)
        request_word = Cat(bus.dat_w, bus.adr, bus.sel, bus.we)
        violation    = Signal()
        self.comb += [
            accepted.eq(bus.cyc & bus.stb & ~bus.stall),
            violation.eq(stalled & (~bus.cyc | ~bus.stb | (request_word != stalled_word))),
            cdc.sink.data.eq(bus.dat_w),
            cdc.sink.adr.eq(bus.adr),
            cdc.sink.sel.eq(bus.sel),
        ]
        sync_input = getattr(self.sync, cdc.input_cd)
        sync_input += [
            bus.ack.eq(accepted & bus.we),
            bus.err.eq(accepted & ~bus.we),
            If(bus.cyc & bus.stb & bus.stall, stats.stalls.eq(stats.stalls + 1)),
            If(violation, stats.overflow.eq(stats.overflow + 1)),
        ]
        ingress.act("IDLE",
            bus.stall.eq(1),
            NextValue(stalled, 0),
            NextValue(frame_bad, 0),
            If(bus.cyc, NextState("DATA")),
        )
        ingress.act("DATA",
            bus.stall.eq(~cdc.sink.ready),
            cdc.sink.valid.eq(bus.cyc & bus.stb & bus.we),
            NextValue(stalled, bus.cyc & bus.stb & bus.stall),
            NextValue(stalled_word, request_word),
            If(violation | (accepted & (~bus.we | ((bus.adr == 2) & bus.dat_w[1]) |
                ((bus.adr == 0) & (bus.sel == 0)))),
                NextValue(frame_bad, 1),
            ),
            If(~bus.cyc,
                NextValue(stalled, 0),
                NextState("END"),
            ),
        )
        ingress.act("END",
            bus.stall.eq(1),
            cdc.sink.valid.eq(1),
            cdc.sink.end.eq(1),
            cdc.sink.error.eq(frame_bad),
            If(cdc.sink.ready,
                NextValue(stats.packets, stats.packets + 1),
                If(frame_bad, NextValue(stats.errors, stats.errors + 1)),
                NextState("IDLE"),
            ),
        )

        # Parse the buffered records in the application clock domain.
        word  = stream.Endpoint([("data", 16), ("sel", 2), ("error", 1)])
        data  = Signal(16)
        sel   = Signal(2)
        first = Signal()
        error = Signal()
        self.fsm = parser = ClockDomainsRenamer(cdc.output_cd)(FSM(reset_state="EMPTY"))
        is_data = (cdc.source.adr == 0) & ~cdc.source.end & (cdc.source.sel != 0)
        status_error = ((cdc.source.adr == 2) & cdc.source.data[1]) | cdc.source.error
        self.comb += [
            word.data.eq(data),
            word.sel.eq(sel),
            word.first.eq(first),
        ]
        parser.act("EMPTY",
            cdc.source.ready.eq(1),
            If(cdc.source.valid,
                If(cdc.source.end,
                    NextValue(error, 0),
                ).Elif(is_data,
                    NextValue(data, cdc.source.data),
                    NextValue(sel, cdc.source.sel),
                    NextValue(first, 1),
                    NextState("WORD"),
                ).Else(
                    NextValue(error, error | status_error | (cdc.source.adr == 0)),
                ),
            ),
        )
        parser.act("WORD",
            If(cdc.source.valid,
                If(is_data | cdc.source.end,
                    word.valid.eq(1),
                    word.last.eq(cdc.source.end),
                    word.error.eq(error | cdc.source.error),
                    cdc.source.ready.eq(word.ready),
                    If(word.ready,
                        If(cdc.source.end,
                            NextValue(error, 0),
                            NextState("EMPTY"),
                        ).Else(
                            NextValue(data, cdc.source.data),
                            NextValue(sel, cdc.source.sel),
                            NextValue(first, 0),
                        ),
                    ),
                ).Else(
                    cdc.source.ready.eq(1),
                    NextValue(error, error | status_error | (cdc.source.adr == 0)),
                ),
            ),
        )

        # Serialize high byte first; SEL=10 and SEL=01 contain one byte.
        self.converter = converter = ClockDomainsRenamer(cdc.output_cd)(FSM(reset_state="HIGH"))
        self.comb += [
            source.valid.eq(word.valid),
            source.error.eq(word.error & source.last),
        ]
        converter.act("HIGH",
            source.data.eq(Mux(word.sel == 1, word.data[:8], word.data[8:])),
            source.first.eq(word.first),
            source.last.eq(word.last & (word.sel != 3)),
            word.ready.eq(source.ready & (word.sel != 3)),
            If(source.valid & source.ready & (word.sel == 3), NextState("LOW")),
        )
        converter.act("LOW",
            source.data.eq(word.data[:8]),
            source.last.eq(word.last),
            word.ready.eq(source.ready),
            If(source.valid & source.ready, NextState("HIGH")),
        )
