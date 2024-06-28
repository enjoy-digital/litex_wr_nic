# Copyright (C) 2024 Enjoy-Digital.

from migen import *

from litex.soc.interconnect import stream

from litex.soc.interconnect import wishbone

# White Rabbit Fabric Stream 2 Wishbone ------------------------------------------------------------

# FIXME: Check Latency for Wishbone Streaming.

class Stream2Wishbone(Module):
    def __init__(self, cd_to="wr"):
        self.sink = sink = stream.Endpoint([("data", 8)])
        self.bus  = bus  = wishbbone.Interface(data_width=16, address_width=2)

        # # #

        # 8-bit to 16-bit Converter.
        self.converter = converter = stream.Converter(8, 16)

        # Clock Domain Crossing.
        self.cdc = cdc = stream.ClockDomainCrossing(
            layout  = [("data", 16)],
            depth   = 16,
            cd_from = "sys",
            cd_to   = cd_to,
        )

        # Sink -> Converter -> CDC.
        self.submodules += Pipeline(sink, converter, cdc)

        # FSM.
        self.fsm = fsm = ClockDomainsRenamer()(FSM(reset_state="IDLE"))
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
            bus.dat_w.eq(0x0200),
            If(bus.ack,
                NextState("DATA")
            )
        )
        fsm.act("WRITE",
            bus.cyc.eq(1),
            bus.stb.eq(cdc.source.valid),
            bus.we.eq(1),
            bus.adr.eq(0b00), # Regular Data.
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
