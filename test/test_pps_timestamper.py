#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import pytest

from migen import *

from litex_wr_nic.gateware.pps_timestamper import PPSTimestamper

# Helpers ------------------------------------------------------------------------------------------

def _dut(width=8):
    dut = Module()
    samples    = Signal(width)
    tm_seconds = Signal(40)
    tm_cycles  = Signal(28)
    tm_valid   = Signal()
    dut.submodules.ts = ts = PPSTimestamper(samples, tm_seconds, tm_cycles, tm_valid, cd="sys")
    dut.samples, dut.seconds, dut.cycles, dut.valid, dut.ts = (
        samples, tm_seconds, tm_cycles, tm_valid, ts)
    return dut


def _feed(dut, words, seconds=5, start_cycle=0, valid=1):
    """Present one sample word per cycle with an advancing WR time."""
    yield dut.seconds.eq(seconds)
    yield dut.valid.eq(valid)
    for index, word in enumerate(words):
        yield dut.cycles.eq(start_cycle + index)
        yield dut.samples.eq(word)
        yield
    # Let the last word and the status synchronizers settle.
    for _ in range(4):
        yield

# Timestamper Tests --------------------------------------------------------------------------------

@pytest.mark.parametrize("index", range(8))
def test_edge_position_within_a_cycle_is_reported(index):
    dut = _dut()

    def stimulus():
        # A rising edge at sample `index`: every later sample is high.
        word = ((1 << (8 - index)) - 1) << index
        yield from _feed(dut, [0x00, word, 0xff, 0xff], seconds=7, start_cycle=100)
        assert (yield dut.ts._count.status) == 1
        assert (yield dut.ts._seconds.status) == 7
        # The word is registered before the timestamper, so the recorded
        # cycle is the one in which the samples were presented.
        assert (yield dut.ts._cycles.status) == 101
        assert (yield dut.ts._fine.fields.index) == index
        assert (yield dut.ts._fine.fields.valid) == 1

    run_simulation(dut, stimulus())


def test_edge_across_a_cycle_boundary_uses_the_first_sample():
    dut = _dut()

    def stimulus():
        # The line goes high on the last sample of a cycle and stays high.
        yield from _feed(dut, [0x00, 0x80, 0xff, 0xff], seconds=1, start_cycle=7)
        assert (yield dut.ts._count.status) == 1
        assert (yield dut.ts._cycles.status) == 8
        assert (yield dut.ts._fine.fields.index) == 7
        # The following cycle is entirely high: no second edge is reported.
        assert (yield dut.ts._count.status) == 1

    run_simulation(dut, stimulus())


def test_each_pulse_is_counted_once_and_the_time_is_updated():
    dut = _dut()

    def stimulus():
        pulses = [0x00, 0x0f, 0x00, 0x00, 0xf0, 0x00]
        yield from _feed(dut, pulses, seconds=2, start_cycle=0)
        assert (yield dut.ts._count.status) == 2
        assert (yield dut.ts._cycles.status) == 4
        assert (yield dut.ts._fine.fields.index) == 4
        # Counting is cleared through the clear input (the CSR pulse field
        # drives the same synchronizer).
        yield dut.ts.clear.eq(1)
        yield
        yield dut.ts.clear.eq(0)
        # The pulse crosses to the timestamping domain and back through the
        # status register.
        for _ in range(8):
            yield
        assert (yield dut.ts._count.status) == 0

    run_simulation(dut, stimulus())


def test_invalid_wr_time_is_flagged_and_raw_samples_are_visible():
    dut = _dut()

    def stimulus():
        yield from _feed(dut, [0x00, 0xff], valid=0)
        assert (yield dut.ts._count.status) == 1
        assert (yield dut.ts._fine.fields.valid) == 0
        # The raw sample word is readable between edges.
        yield dut.samples.eq(0xa5)
        for _ in range(4):
            yield
        assert (yield dut.ts._samples.status) == 0xa5

    run_simulation(dut, stimulus())


def test_a_single_sample_per_cycle_is_supported():
    """One bit per cycle: the edge is resolved to the cycle, index stays 0."""
    dut = _dut(width=1)

    def stimulus():
        yield from _feed(dut, [0b0, 0b1, 0b1, 0b0, 0b1], seconds=3, start_cycle=40)
        assert (yield dut.ts._count.status) == 2
        assert (yield dut.ts._cycles.status) == 44
        assert (yield dut.ts._fine.fields.index) == 0
        assert (yield dut.ts._seconds.status) == 3

    run_simulation(dut, stimulus())
