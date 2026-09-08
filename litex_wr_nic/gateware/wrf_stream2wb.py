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

# White Rabbit Fabric Stream to Wishbone -----------------------------------------------------------

class Stream2Wishbone(LiteXModule):
    """Transmit packets with independent pipelined request/response handling.

    STALL governs request acceptance; ACK retires accepted requests. CYC stays
    asserted through the final response. ERR/RTY abort the packet and drain its
    remaining input. An input error produces a trailing WR error status word.
    """
    def __init__(self, cd_to="wr", depth=16, max_pending=16, match_class=0x02):
        if max_pending < 1 or not 0 <= match_class <= 255:
            raise ValueError("WR fabric needs a positive pending limit and an 8-bit class.")
        self.sink = sink = stream.Endpoint([("data", 8), ("error", 1)])
        self.bus  = bus  = WRFInterface()

        # # #

        # Pack the error flag alongside each byte so it follows the exact same
        # buffering and packet boundaries as the data, including odd packets.
        self.cdc = cdc = WRFClockCrossing(
            [("data", 18), ("valid_token_count", 2)], "sys", cd_to, depth)
        self.converter = converter = ClockDomainsRenamer(cdc.input_cd)(
            stream.Converter(9, 18, reverse=True, report_valid_token_count=True))
        self.comb += [
            sink.connect(converter.sink, omit={"data", "error"}),
            converter.sink.data.eq(Cat(sink.data, sink.error)),
            converter.source.connect(cdc.sink),
        ]
        self.stats = stats = WRFCounters(cdc.output_cd)
        self.fsm = fsm = ClockDomainsRenamer(cdc.output_cd)(FSM(reset_state="IDLE"))
        pending = Signal(max=max_pending + 1)
        accepted = Signal()
        response = Signal()
        failed = Signal()
        room = Signal()
        frame_bad = Signal()
        data_word = Cat(cdc.source.data[:8], cdc.source.data[9:17])
        word_error = cdc.source.data[17] | ((cdc.source.valid_token_count == 2) & cdc.source.data[8])
        self.comb += [
            accepted.eq(bus.cyc & bus.stb & ~bus.stall),
            response.eq(bus.cyc & (bus.ack | bus.err | bus.rty) & ((pending != 0) | accepted)),
            failed.eq(response & (bus.err | bus.rty)),
            room.eq(pending < max_pending),
            bus.we.eq(1),
        ]
        sync_output = getattr(self.sync, cdc.output_cd)
        sync_output += [
            If(accepted & ~response, pending.eq(pending + 1)),
            If(response & ~accepted, pending.eq(pending - 1)),
            If(~bus.cyc, pending.eq(0)),
            If(bus.cyc & bus.stb & bus.stall, stats.stalls.eq(stats.stalls + 1)),
        ]
        fsm.act("IDLE",
            NextValue(frame_bad, 0),
            If(cdc.source.valid, NextState("STATUS")),
        )
        fsm.act("STATUS",
            bus.cyc.eq(1),
            bus.stb.eq(room),
            bus.adr.eq(2),
            bus.sel.eq(3),
            bus.dat_w.eq(match_class << 8),
            If(accepted, NextState("DATA")),
            If(failed,
                NextValue(frame_bad, 1),
                NextState("DROP"),
            ),
        )
        fsm.act("DATA",
            bus.cyc.eq(1),
            bus.stb.eq(cdc.source.valid & room),
            bus.adr.eq(0),
            bus.sel.eq(Mux(cdc.source.valid_token_count == 1, 2, 3)),
            bus.dat_w.eq(data_word),
            cdc.source.ready.eq(accepted),
            If(accepted,
                NextValue(frame_bad, frame_bad | word_error),
                If(cdc.source.last,
                    If(frame_bad | word_error,
                        NextState("ERROR_STATUS"),
                    ).Else(
                        NextState("WAIT"),
                    ),
                ),
            ),
            If(failed,
                NextValue(frame_bad, 1),
                If(accepted & cdc.source.last, NextState("END")).Else(NextState("DROP")),
            ),
        )
        fsm.act("ERROR_STATUS",
            bus.cyc.eq(1),
            bus.stb.eq(room),
            bus.adr.eq(2),
            bus.sel.eq(3),
            bus.dat_w.eq((match_class << 8) | 2),
            If(accepted, NextState("WAIT")),
            If(failed, NextState("END")),
        )
        fsm.act("WAIT",
            bus.cyc.eq(1),
            If(pending == 0, NextState("END")),
            If(failed,
                NextValue(frame_bad, 1),
                NextState("END"),
            ),
        )
        fsm.act("DROP",
            # ERR/RTY terminate the Wishbone cycle. Discard the rest of this
            # application packet before accepting a new frame.
            cdc.source.ready.eq(1),
            If(cdc.source.valid & cdc.source.last, NextState("END")),
        )
        fsm.act("END",
            NextValue(stats.packets, stats.packets + 1),
            If(frame_bad, NextValue(stats.errors, stats.errors + 1)),
            NextState("RECOVER"),
        )
        fsm.act("RECOVER",
            If(~(bus.ack | bus.err | bus.rty), NextState("IDLE")),
        )
