# Copyright (C) 2024 Enjoy-Digital.

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from litex.soc.interconnect import wishbone

# White Rabbit Fabric Stream 2 Wishbone ------------------------------------------------------------

# FIXME: Check Latency for Wishbone Streaming.
# FIXME: Check when non-multiple of 16-bit (last word).

class Wishbone2Stream(LiteXModule):
    def __init__(self, cd_from="wr"):
        self.bus    = bus    = wishbone.Interface(data_width=16, address_width=2, addressing="byte")
        self.source = source = stream.Endpoint([("data", 8)])

        # # #

        # Signals.
        valid = Signal()
        last  = Signal()
        data  = Signal(16)

        # Always accept incoming accesses.
        self.comb += bus.ack.eq(1)

        # Clock Domain Crossing.
        self.cdc = cdc = stream.ClockDomainCrossing(
            layout  = [("data", 16)],
            depth   = 16,
            cd_from = cd_from,
            cd_to   = "sys",
        )

        # FSM.
        self.fsm = fsm = ClockDomainsRenamer(cd_from)(FSM(reset_state="IDLE"))
        fsm.act("IDLE",
            NextValue(valid, 0),
            NextValue(data,  0),
            If(bus.stb & bus.cyc,
                # On Status Word, jump to DATA.
                If(bus.adr == 0b10,
                    NextState("DATA")
                )
            )
        )
        fsm.act("DATA",
            # Copy Regular Data.
            If(bus.stb & bus.cyc,
                If(bus.adr == 0b00,
                    NextValue(valid, 1),
                    NextValue(data, bus.dat_w),
                )
            ),
            # Return to IDLE when Regular Data or Access is done.
            If(~bus.cyc | (bus.adr != 0b00),
                last.eq(1),
                NextState("IDLE")
            )
        )
        self.comb += cdc.sink.last.eq(last)
        self.sync += [
            cdc.sink.valid.eq(valid),
            cdc.sink.data.eq(data),
        ]

        # 16-bit to 8-bit Converter.
        self.converter = converter = stream.Converter(16, 8, reverse=True)

        # CDC -> Converter -> Source.
        self.submodules += stream.Pipeline(cdc, converter, source)

