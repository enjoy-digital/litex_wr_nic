#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg, BusSynchronizer

from litex.gen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import CSR, CSRStatus

# Application Time ---------------------------------------------------------------------------------

WR_TIME_INVALID  = 0
WR_TIME_VALID    = 1
WR_TIME_HOLDOVER = 2

WR_TIME_LAYOUT = [
    ("seconds",    40),
    ("cycles",     28),
    ("time_valid",  1),
    ("link_up",     1),
    ("state",       2),
]

WR_TICKS_PER_SECOND = 62_500_000


class WRApplicationTime(LiteXModule):
    """Expose native WR time, with an explicit policy for loss of link.

    Timecode and validity are synchronous to cd_time (normally the 62.5 MHz
    PHY reference clock). Link status is synchronized from the control domain.
    VALID means the core marks time valid and the link is up; it is not an
    independent measurement of PTP servo lock or timing accuracy.
    """
    def __init__(self, seconds, cycles, valid, link_up, cd_time="wr", allow_holdover=False,
        with_csr=True):
        self.time = Record(WR_TIME_LAYOUT)

        # # #

        link = Signal()
        self.specials += MultiReg(link_up, link, cd_time)
        self.comb += [
            self.time.seconds.eq(seconds),
            self.time.cycles.eq(cycles),
            self.time.link_up.eq(link),
            self.time.time_valid.eq(valid & (link | int(allow_holdover))),
            self.time.state.eq(WR_TIME_INVALID),
            If(valid,
                If(link,
                    self.time.state.eq(WR_TIME_VALID),
                ).Elif(allow_holdover,
                    self.time.state.eq(WR_TIME_HOLDOVER),
                ),
            ),
        ]
        if with_csr:
            self.add_csr(cd_time)

    def add_csr(self, cd_time):
        self._capture    = CSR() # Write to request a coherent WR time snapshot.
        self._busy       = CSRStatus(description="A snapshot request is pending.")
        self._done       = CSRStatus(description="The snapshot is ready; cleared on recapture or reset.")
        self._seconds    = CSRStatus(40, description="Captured WR TAI seconds.")
        self._cycles     = CSRStatus(28, description="Captured 16 ns cycles within the second.")
        self._time_valid = CSRStatus(description="Captured time validity after applying link-loss policy.")
        self._link_up    = CSRStatus(description="Captured WR link status.")
        self._state      = CSRStatus(2, description="Captured state: 0 = invalid, 1 = valid, 2 = holdover.")

        # # #

        request   = Signal()
        pending   = Signal()
        captured  = Record(WR_TIME_LAYOUT)
        reset_sys = Signal()
        self.specials += MultiReg(ResetSignal(cd_time), reset_sys)
        self.request_cdc = stream.ClockDomainCrossing([("data", 1)],
            cd_from="sys", cd_to=cd_time, depth=4, with_common_rst=True)
        self.reply_cdc = stream.ClockDomainCrossing(WR_TIME_LAYOUT,
            cd_from=cd_time, cd_to="sys", depth=4, with_common_rst=True)
        self.comb += [
            self.request_cdc.sink.valid.eq(request),
            self.request_cdc.source.ready.eq(~pending),
            self.reply_cdc.sink.valid.eq(pending),
            self.reply_cdc.source.ready.eq(1),
        ]
        for name, _ in WR_TIME_LAYOUT:
            self.comb += getattr(self.reply_cdc.sink, name).eq(getattr(captured, name))
        sync_time = getattr(self.sync, cd_time)
        sync_time += [
            If(self.request_cdc.source.valid & self.request_cdc.source.ready,
                captured.raw_bits().eq(self.time.raw_bits()),
                pending.eq(1),
            ).Elif(self.reply_cdc.sink.valid & self.reply_cdc.sink.ready,
                pending.eq(0),
            ),
        ]
        self.sync += [
            If(request & self.request_cdc.sink.ready, request.eq(0)),
            If(self._capture.wr_stb & ~self._busy.status,
                request.eq(1),
                self._busy.status.eq(1),
                self._done.status.eq(0),
            ),
            If(self.reply_cdc.source.valid,
                self._busy.status.eq(0),
                self._done.status.eq(1),
                *[getattr(self, "_" + name).status.eq(getattr(self.reply_cdc.source, name))
                    for name, _ in WR_TIME_LAYOUT],
            ),
            If(reset_sys,
                request.eq(0),
                self._busy.status.eq(0),
                self._done.status.eq(0),
                self._time_valid.status.eq(0),
                self._state.status.eq(WR_TIME_INVALID),
            ),
        ]

# Event Timestamping -------------------------------------------------------------------------------

class WREventTimestamp(LiteXModule):
    """Timestamp event pulses in cd_time and queue records for an application.

    The pulse and tag must already be synchronous to cd_time. Crossing an
    asynchronous event into that domain adds synchronizer latency/uncertainty.
    Invalid timestamps are delivered with valid=0, not silently discarded.
    """
    def __init__(self, time, tag_width=8, depth=16, cd_time="wr", cd_out="sys"):
        self.event   = Signal()
        self.tag     = Signal(tag_width)
        self.dropped = Signal(32)
        self.source = stream.Endpoint(WR_TIME_LAYOUT + [("tag", tag_width)])

        # # #

        layout = WR_TIME_LAYOUT + [("tag", tag_width)]
        if cd_time == cd_out:
            self.queue = ClockDomainsRenamer(cd_time)(stream.SyncFIFO(layout, depth, buffered=True))
        else:
            self.queue = stream.ClockDomainCrossing(layout,
                cd_from=cd_time, cd_to=cd_out, depth=depth, with_common_rst=True)
        self.comb += [
            self.queue.sink.valid.eq(self.event),
            self.queue.sink.first.eq(1),
            self.queue.sink.last.eq(1),
            self.queue.sink.tag.eq(self.tag),
            self.queue.source.connect(self.source),
            *[getattr(self.queue.sink, name).eq(getattr(time, name)) for name, _ in WR_TIME_LAYOUT],
        ]
        sync_time = getattr(self.sync, cd_time)
        sync_time += If(self.event & ~self.queue.sink.ready, self.dropped.eq(self.dropped + 1))
        self._dropped = CSRStatus(32,
            description="Events dropped while the timestamp queue was full, wrapping at 32 bits.")
        self.dropped_cdc = BusSynchronizer(32, cd_time, "sys")
        self.comb += [
            self.dropped_cdc.i.eq(self.dropped),
            self._dropped.status.eq(self.dropped_cdc.o),
        ]

# Scheduled Trigger -------------------------------------------------------------------------------

class WRTimeTrigger(LiteXModule):
    """One scheduled trigger in the time domain, with explicit cancellation.

    sink, pulse, pending, missed and cancelled use cd_time. A command must
    specify a future native WR timestamp. Past times, forward steps over the
    target and loss of validity never generate a late trigger.
    """
    def __init__(self, time, cd_time="wr", ticks_per_second=WR_TICKS_PER_SECOND):
        self.sink      = stream.Endpoint([("seconds", 40), ("cycles", 28)])
        self.pulse     = Signal()
        self.pending   = Signal()
        self.missed    = Signal(32)
        self.cancelled = Signal(32)

        # # #

        target    = Signal(68)
        previous  = Signal(68)
        now       = Cat(time.cycles, time.seconds)
        requested = Cat(self.sink.cycles, self.sink.seconds)
        self.comb += [
            self.sink.ready.eq(~self.pending),
            self.pulse.eq(self.pending & time.time_valid & (now == target)),
        ]
        sync_time = getattr(self.sync, cd_time)
        sync_time += [
            previous.eq(now),
            If(self.sink.valid & self.sink.ready,
                If(~time.time_valid,
                    self.cancelled.eq(self.cancelled + 1),
                ).Elif((requested <= now) | (self.sink.cycles >= ticks_per_second),
                    self.missed.eq(self.missed + 1),
                ).Else(
                    target.eq(requested),
                    self.pending.eq(1),
                ),
            ),
            If(self.pending,
                If(~time.time_valid | (now < previous),
                    self.pending.eq(0),
                    self.cancelled.eq(self.cancelled + 1),
                ).Elif(now > target,
                    self.pending.eq(0),
                    self.missed.eq(self.missed + 1),
                ).Elif(self.pulse,
                    self.pending.eq(0),
                ),
            ),
        ]
