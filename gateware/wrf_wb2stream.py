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

# White Rabbit Fabric Wishbone 2 Stream ------------------------------------------------------------
#
# This module converts 16-bit Wishbone bus transactions (wr_clk) to a 8-bit stream (sys_clk):
#                                          │
#                           ◄──    wr_clk  │ sys_clk ──►
#                                          │
#                         ┌────────┐    ┌───────┐  ┌─────────────────┐
#                         │        │    │       │  │ 16-bit to 8-bit │
#     16-bit Wishbone ────►  FSM   ┼────►  CDC  ┼──►                 ┼───► 8-bit Stream
#                         │        │    │       │  │    Converter    │
#                         └────────┘    └───────┘  └─────────────────┘

class Wishbone2Stream(LiteXModule):
    def __init__(self, cd_from="wr"):
        self.bus    = bus    = wishbone.Interface(data_width=16, address_width=2, addressing="byte")
        self.source = source = stream.Endpoint([("data", 8)])

        # # #

        # Signals.
        valid = Signal()
        last  = Signal()
        sel   = Signal(2)
        data  = Signal(16)

        # Always accept incoming accesses.
        self.comb += bus.ack.eq(1)

        # Clock Domain Crossing.
        self.cdc = cdc = stream.ClockDomainCrossing(
            layout  = [("data", 16), ("sel", 2)],
            cd_from = cd_from,
            cd_to   = "sys",
            depth   = 16,
            with_common_rst = True,
        )

        # FSM.
        self.fsm = fsm = ClockDomainsRenamer(cd_from)(FSM(reset_state="IDLE"))
        fsm.act("IDLE",
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
                    valid.eq(1),
                    sel.eq(bus.sel),
                    data.eq(bus.dat_w),
                )
            ),
            # Return to IDLE when Regular Data or Access is done.
            If(~bus.cyc | (bus.stb & bus.cyc & (bus.adr != 0b00)),
                last.eq(1),
                NextState("IDLE")
            )
        )
        self.comb += cdc.sink.last.eq(last)
        _sync = getattr(self.sync, cd_from)
        _sync += [
            cdc.sink.valid.eq(valid),
            cdc.sink.sel.eq(sel),
            cdc.sink.data.eq(data),
        ]

        # 16-bit to 8-bit Converter.
        self.converter = converter = stream.Converter(16, 8, reverse=True)

        # CDC -> Converter -> Source.
        self.comb += [
            If(cdc.source.valid,
                # Even number of bytes.
                If(cdc.source.sel == 0b11,
                    cdc.source.connect(converter.sink, omit={"sel"}),
                    converter.source.connect(source),
                # Odd number of bytes.
                ).Elif(cdc.source.sel == 0b10,
                    cdc.source.connect(source, omit={"sel", "data"}),
                    source.data.eq(cdc.source.data[8:16])
                ).Else(
                    cdc.source.ready.eq(1), # Ready by default.
                )
            )
        ]
