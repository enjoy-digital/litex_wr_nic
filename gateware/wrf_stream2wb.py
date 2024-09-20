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

# White Rabbit Fabric Stream 2 Wishbone ------------------------------------------------------------

# FIXME: Check Latency for Wishbone Streaming.

class Stream2Wishbone(LiteXModule):
    def __init__(self, cd_to="wr"):
        self.sink = sink = stream.Endpoint([("data", 8), ("last_be", 1), ("error", 1)])  # CHECKME: last_be, error.
        self.bus  = bus  = wishbone.Interface(data_width=16, address_width=2, addressing="byte")

        # # #

        # 8-bit to 16-bit Converter.
        self.converter = converter = stream.StrideConverter(
            description_from = [("data",  8), ("sel", 1)],
            description_to   = [("data", 16), ("sel", 2)],
            reverse          = True,
        )

        # Clock Domain Crossing.
        self.cdc = cdc = stream.ClockDomainCrossing(
            layout  = [("data", 16), ("sel", 2)],
            depth   = 16,
            cd_from = "sys",
            cd_to   = cd_to,
        )

        # Sink -> Converter -> CDC.
        self.comb += [
            sink.connect(converter.sink, omit={"last_be", "error"}), #  CHECKME: last_be, error.
            converter.sink.sel.eq(1),
            converter.source.connect(cdc.sink),
        ]

        # FSM.
        self.fsm = fsm = ClockDomainsRenamer(cd_to)(FSM(reset_state="IDLE"))
        fsm.act("IDLE",
            If(cdc.source.valid,
                NextState("STATUS")
            )
        )
        fsm.act("STATUS",
            bus.cyc.eq(1),
            bus.stb.eq(1),
            bus.we.eq(1),
            bus.adr.eq(0b10), # Status Word.
            bus.sel.eq(0b11),
            bus.dat_w.eq(0x0200),
            If(bus.ack,
                NextState("DATA")
            )
        )
        fsm.act("DATA",
            bus.cyc.eq(1),
            bus.stb.eq(cdc.source.valid),
            bus.we.eq(1),
            bus.adr.eq(0b00), # Regular Data.
            bus.sel.eq(cdc.source.sel),
            bus.dat_w.eq(cdc.source.data),
            If(bus.ack,
                cdc.source.ready.eq(1),
                If(cdc.source.last,
                    NextState("END")
                )
            )
        )
        fsm.act("END",
            NextState("IDLE")
        )
