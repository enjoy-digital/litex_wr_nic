# Copyright (C) 2024 Enjoy-Digital.

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from litex.soc.interconnect import wishbone

# White Rabbit Fabric Stream 2 Wishbone ------------------------------------------------------------

class UDPDummyGenerator(LiteXModule):
    def __init__(self):
        self.send   = Signal()
        self.source = source = stream.Endpoint([("data", 8)])

        # # #

        # UDP Packet.
        packet = [
            0x74, 0x56, 0x3C, 0x4F, 0x4C, 0x6D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x08, 0x00, 0x45, 0x00, 0x00, 0xEC, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x11,
            0xF7, 0x9A, 0xC0, 0xA8, 0x01, 0x05, 0xC0, 0xA8, 0x01, 0x7A, 0x10, 0x00,
            0x10, 0x00, 0x00, 0xD8, 0x00, 0x00, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34,
            0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34,
            0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34,
            0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34,
            0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34,
            0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34,
            0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34,
            0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34,
            0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34,
            0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34,
            0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34,
            0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34,
        ]

        # Memory.
        mem  = Memory(8, len(packet), init=packet)
        port = mem.get_port(async_read=True)
        self.specials += mem, port

        # FSM.
        count = Signal(16)
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            NextValue(count, 0),
            If(self.send,
                NextState("SEND")
            )
        )
        fsm.act("SEND",
            port.adr.eq(count),
            source.valid.eq(1),
            source.data.eq(port.dat_r),
            source.last.eq(count == (len(packet) - 1)),
            If(source.ready,
                NextValue(count, count + 1),
                If(source.last,
                    NextState("IDLE")
                )
            )
        )

# FIXME: Check Latency for Wishbone Streaming.

class Stream2Wishbone(LiteXModule):
    def __init__(self, cd_to="wr"):
        self.sink = sink = stream.Endpoint([("data", 8)])
        self.bus  = bus  = wishbone.Interface(data_width=16, address_width=2, addressing="byte")

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
        self.submodules += stream.Pipeline(sink, converter, cdc)

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
            bus.sel.eq(0b11),
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
