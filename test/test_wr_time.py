#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.fhdl.structure import _Assign

from litex.gen import LiteXModule

from litex_wr_nic.gateware.wr_time import (
    WRApplicationTime, WREventTimestamp, WRTimeTrigger, WR_TIME_LAYOUT,
    WR_TIME_INVALID, WR_TIME_VALID, WR_TIME_HOLDOVER,
)

# Simulation Helpers -------------------------------------------------------------------------------

def simulate(dut, generators):
    clocks = {"sys": 10, "wr": 16}
    for name in clocks:
        if not hasattr(dut, "cd_" + name):
            setattr(dut, "cd_" + name, ClockDomain(name))
    fragment = dut.get_fragment()
    for statement in fragment.comb:
        if isinstance(statement, _Assign) and isinstance(statement.r, ClockSignal):
            if statement.r.cd in clocks:
                for domain in fragment.clock_domains:
                    if statement.l is domain.clk:
                        clocks[domain.name] = clocks[statement.r.cd]
    run_simulation(fragment, generators, clocks=clocks)

# Application Time Tests ---------------------------------------------------------------------------

def test_link_loss_policy_is_explicit():
    for holdover in (False, True):
        valid = Signal(reset=1)
        link  = Signal(reset=1)
        dut = WRApplicationTime(1, 2, valid, link, allow_holdover=holdover, with_csr=False)

        def check():
            for _ in range(5):
                yield
            assert (yield dut.time.state) == WR_TIME_VALID
            yield link.eq(0)
            for _ in range(5):
                yield
            assert (yield dut.time.time_valid) == int(holdover)
            assert (yield dut.time.state) == (WR_TIME_HOLDOVER if holdover else WR_TIME_INVALID)
            yield valid.eq(0)
            yield
            assert (yield dut.time.time_valid) == 0
            assert (yield dut.time.state) == WR_TIME_INVALID

        simulate(dut, {"wr": check()})


def test_snapshot_is_coherent_at_seconds_rollover_and_stable_until_recapture():
    dut = LiteXModule()
    dut.cd_sys = ClockDomain("sys")
    dut.cd_wr  = ClockDomain("wr")
    seconds = Signal(40, reset=9)
    cycles  = Signal(28, reset=62_499_995)
    dut.time = WRApplicationTime(seconds, cycles, 1, 1)
    dut.sync.wr += If(cycles == 62_499_999,
        cycles.eq(0),
        seconds.eq(seconds + 1),
    ).Else(
        cycles.eq(cycles + 1),
    )
    observed = []

    @passive
    def reference():
        while True:
            observed.append(((yield seconds), (yield cycles)))
            yield

    def host():
        yield dut.time._capture.wr_stb.eq(1)
        yield
        yield dut.time._capture.wr_stb.eq(0)
        for _ in range(200):
            if (yield dut.time._done.status):
                break
            yield
        else:
            raise AssertionError("Snapshot did not complete")
        result = ((yield dut.time._seconds.status), (yield dut.time._cycles.status))
        assert result in observed
        assert (yield dut.time._busy.status) == 0
        for _ in range(30):
            yield
            assert ((yield dut.time._seconds.status), (yield dut.time._cycles.status)) == result
        # Reset the time domain while sys continues: clear the stale validity.
        yield dut.cd_wr.rst.eq(1)
        for _ in range(10):
            yield
        assert (yield dut.time._done.status) == 0
        assert (yield dut.time._time_valid.status) == 0

    simulate(dut, {"sys": host(), "wr": reference()})


def test_event_queue_preserves_flags_and_reports_loss_under_backpressure():
    time = Record(WR_TIME_LAYOUT, name="time")
    dut = WREventTimestamp(time, depth=4)
    received = []

    def events():
        for _ in range(8):
            yield
        yield time.seconds.eq(12)
        yield time.time_valid.eq(1)
        yield time.state.eq(WR_TIME_VALID)
        # Keep the consumer stopped long enough to fill the async FIFO.
        for number in range(12):
            yield dut.event.eq(1)
            yield dut.tag.eq(number)
            yield time.cycles.eq(number)
            yield
        yield dut.event.eq(0)
        for _ in range(100):
            yield
        assert (yield dut.dropped) == 8
        # Invalid time is retained as an explicit flag on the event.
        yield time.time_valid.eq(0)
        yield time.state.eq(WR_TIME_INVALID)
        yield time.cycles.eq(99)
        yield dut.tag.eq(99)
        yield dut.event.eq(1)
        yield
        yield dut.event.eq(0)
        for _ in range(100):
            yield

    def consumer():
        for _ in range(40):
            yield
        yield dut.source.ready.eq(1)
        for _ in range(350):
            if (yield dut.source.valid) and (yield dut.source.ready):
                received.append((
                    (yield dut.source.tag),
                    (yield dut.source.cycles),
                    (yield dut.source.time_valid),
                ))
                assert (yield dut.source.first) and (yield dut.source.last)
            yield

    simulate(dut, {"wr": events(), "sys": consumer()})
    assert received == [(n, n, 1) for n in range(4)] + [(99, 99, 0)]


def test_trigger_fires_once_and_cancels_invalid_past_or_skipped_times():
    time = Record(WR_TIME_LAYOUT, name="time")
    dut = WRTimeTrigger(time)

    def schedule(second, cycle):
        yield dut.sink.seconds.eq(second)
        yield dut.sink.cycles.eq(cycle)
        yield dut.sink.valid.eq(1)
        yield
        yield dut.sink.valid.eq(0)
        yield

    def check():
        yield time.time_valid.eq(1)
        yield time.seconds.eq(3)
        yield time.cycles.eq(62_499_998)
        yield from schedule(4, 1)
        assert (yield dut.pending)
        yield time.seconds.eq(4)
        yield time.cycles.eq(0)
        yield
        assert not (yield dut.pulse)
        yield time.cycles.eq(1)
        yield
        assert (yield dut.pulse)
        yield time.cycles.eq(2)
        yield
        assert not (yield dut.pulse)
        assert not (yield dut.pending)
        yield from schedule(3, 1)
        assert (yield dut.missed) == 1
        yield from schedule(4, 10)
        yield time.cycles.eq(11)
        for _ in range(2):
            yield
        assert (yield dut.missed) == 2
        assert not (yield dut.pulse)
        yield from schedule(4, 20)
        yield time.time_valid.eq(0)
        for _ in range(2):
            yield
        assert (yield dut.cancelled) == 1
        yield time.time_valid.eq(1)
        yield from schedule(4, 30)
        yield time.seconds.eq(2) # A backwards time step cancels the old request.
        for _ in range(2):
            yield
        assert (yield dut.cancelled) == 2
        assert not (yield dut.pending)

    simulate(dut, {"wr": check()})
