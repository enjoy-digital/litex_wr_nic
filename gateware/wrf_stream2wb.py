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
#
# This module converts a 8-bit stream (sys_clk) into 16-bit Wishbone bus transactions (wr_clk):
#                                                  │
#                                   ◄──  sys_clk   │  wr_clk ──►
#                                                  │
#                        ┌─────────────────┐    ┌───────┐    ┌────────┐
#                        │ 8-bit to 16-bit │    │       │    │        │
#      8-bit Stream  ────►                 ┼────►  CDC  ┼────►  FSM   ┼───► 16-bit Wishbone
#                        │    Converter    │    │       │    │        │
#                        └─────────────────┘    └───────┘    └────────┘

class Stream2Wishbone(LiteXModule):
    def __init__(self, cd_to="wr"):
        self.sink = sink = stream.Endpoint([("data", 8)])
        self.bus  = bus  = wishbone.Interface(data_width=16, address_width=2, addressing="byte")

        # # #

        # 8-bit to 16-bit Converter.
        self.converter = converter = stream.Converter(8, 16, reverse=True, report_valid_token_count=True)

        # Clock Domain Crossing.
        self.cdc = cdc = stream.ClockDomainCrossing(
            layout  = [("data", 16), ("valid_token_count", 3)],
            cd_from = "sys",
            cd_to   = cd_to,
            depth   = 16,
            with_common_rst = True,
        )

        # Sink -> Converter -> CDC.
        self.comb += [
            sink.connect(converter.sink),
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
            Case(cdc.source.valid_token_count, {
                1         : bus.sel.eq(0b10),
                2         : bus.sel.eq(0b11),
                "default" : bus.sel.eq(0b11),
            }),
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
