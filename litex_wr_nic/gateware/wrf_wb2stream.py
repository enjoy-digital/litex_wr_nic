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

        # Retain the final word until another word or the end of the frame
        # arrives. A fabric master may pause between its final data and CYC
        # deassertion, so a one-clock delayed VALID cannot delimit the packet.
        pending       = Signal()
        pending_sel   = Signal(2)
        pending_data  = Signal(16)
        pending_first = Signal()
        first         = Signal()
        frame_error   = Signal()
        self.error    = Signal()
        self.stall    = Signal()

        self.cdc = cdc = stream.ClockDomainCrossing(
            layout          = [("data", 16), ("sel", 2), ("error", 1)],
            cd_from         = cd_from,
            cd_to           = "sys",
            depth           = 16,
            with_common_rst = True,
        )
        self.comb += [
            cdc.sink.data.eq(pending_data),
            cdc.sink.first.eq(pending_first),
            cdc.sink.sel.eq(pending_sel),
            cdc.sink.error.eq(frame_error),
            bus.ack.eq(bus.cyc & bus.stb & ~self.stall),
        ]

        self.fsm = fsm = ClockDomainsRenamer(cd_from)(FSM(reset_state="IDLE"))
        fsm.act("IDLE",
            If(bus.stb & bus.cyc & (bus.adr == 0b10),
                NextValue(frame_error, bus.dat_w[1]),
                NextValue(first, 1),
                NextState("DATA"),
            ),
        )
        fsm.act("DATA",
            If(~bus.cyc | (bus.stb & (bus.adr != 0b00)),
                If(bus.cyc & bus.stb & (bus.adr == 0b10),
                    NextValue(frame_error, frame_error | bus.dat_w[1]),
                ),
                NextState("FLUSH"),
            ).Elif(bus.stb,
                # A new word releases the previous one as non-final data.
                # Honor FIFO backpressure before acknowledging replacement.
                self.stall.eq(pending & ~cdc.sink.ready),
                cdc.sink.valid.eq(pending),
                If(~self.stall,
                    NextValue(pending, 1),
                    NextValue(pending_sel, bus.sel),
                    NextValue(pending_data, bus.dat_w),
                    NextValue(pending_first, first),
                    NextValue(first, 0),
                ),
            ),
        )
        fsm.act("FLUSH",
            # Do not acknowledge a following frame until the final word has
            # crossed. Any trailing OOB words are ignored by IDLE.
            self.stall.eq(1),
            cdc.sink.valid.eq(pending),
            cdc.sink.last.eq(1),
            If(~pending | cdc.sink.ready,
                NextValue(pending, 0),
                NextState("IDLE"),
            ),
        )

        # 16-bit to 8-bit Converter.
        self.converter = converter = stream.Converter(16, 8, reverse=True)

        # Error is sideband to preserve the historical byte-stream layout.
        self.comb += self.error.eq(cdc.source.error)

        # CDC -> Converter -> Source.
        self.comb += [
            If(cdc.source.valid,
                # Even number of bytes.
                If(cdc.source.sel == 0b11,
                    cdc.source.connect(converter.sink, omit={"sel", "error"}),
                    converter.source.connect(source),
                # Odd number of bytes.
                ).Elif(cdc.source.sel == 0b10,
                    cdc.source.connect(source, omit={"sel", "data", "error"}),
                    source.data.eq(cdc.source.data[8:16])
                ).Else(
                    cdc.source.ready.eq(1), # Ready by default.
                )
            )
        ]
