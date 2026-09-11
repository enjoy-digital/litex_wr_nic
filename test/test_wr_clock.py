#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import pytest

from migen import *
from migen.fhdl.structure import _Assign
from migen.sim import passive

from litex.gen import LiteXModule

from litex_wr_nic.gateware.ps_gen import PSGen
from litex_wr_nic.gateware.wr_clock import WRTuningCalibration, WRTuningCDC, WRMMCMBackend

# Simulation Helpers -------------------------------------------------------------------------------

def simulate(dut, generators, clocks=None):
    dut.cd_sys = ClockDomain("sys")
    dut.cd_wr  = ClockDomain("wr")
    dut.cd_ps  = ClockDomain("ps")
    fragment = dut.get_fragment()
    clocks   = dict(clocks or {"sys": 10, "wr": 16, "ps": 6})
    for statement in fragment.comb:
        if isinstance(statement, _Assign) and isinstance(statement.r, ClockSignal):
            if statement.r.cd in clocks:
                for domain in fragment.clock_domains:
                    if statement.l is domain.clk:
                        clocks[domain.name] = clocks[statement.r.cd]
    run_simulation(fragment, generators, clocks=clocks)

# Clock Tuning Tests -------------------------------------------------------------------------------

@pytest.mark.parametrize("calibration,values", [
    ({}, [(0, 0, 0), (32768, 32768, 0), (65535, 65535, 0)]),
    ({"polarity": -1}, [(0, 65535, 1), (32768, 32768, 0), (65535, 1, 0)]),
    ({"gain": 3, "gain_shift": 1, "offset": 100},
        [(0, 0, 1), (32767, 32866, 0), (32768, 32868, 0), (65535, 65535, 1)]),
])
def test_calibration_full_range_and_saturation(calibration, values):
    dut = WRTuningCalibration(cd="sys", **calibration)

    def check():
        for value, expected, clipped in values:
            yield dut.sink.data.eq(value)
            yield dut.sink.load.eq(1)
            yield
            yield dut.sink.load.eq(0)
            yield
            assert (yield dut.source.load) == 1
            assert (yield dut.source.data) == expected
            assert (yield dut.clipped) == clipped
            yield
            assert (yield dut.source.load) == 0
    simulate(dut, check())


def test_tuning_cdc_is_coherent_and_eventually_delivers_latest_command():
    dut = WRTuningCDC(cd_from="wr", cd_to="ps", depth=4)
    sent = [((index * 977) ^ 0xa55a) & 0xffff for index in range(100)]
    received = []
    superseded = []

    def producer():
        for _ in range(10):
            yield
        yield dut.sink.load.eq(1)
        for value in sent:
            yield dut.sink.data.eq(value)
            yield
        yield dut.sink.load.eq(0)
        for _ in range(1000):
            yield
        superseded.append((yield dut.superseded))

    def consumer():
        for _ in range(140):
            if (yield dut.source.load):
                received.append((yield dut.source.data))
            yield
    simulate(dut, {"wr": producer(), "ps": consumer()}, {"sys": 10, "wr": 6, "ps": 60})
    assert received[-1] == sent[-1]
    assert all(value in sent for value in received)
    assert [sent.index(value) for value in received] == sorted(sent.index(value) for value in received)
    assert len(received) + superseded[0] == len(sent)
    assert superseded[0] > 0


def test_mmcm_starts_neutral_and_holds_direction_until_completion():
    dut = WRMMCMBackend("ps", "wr", width=8, timeout_cycles=100)
    directions = []
    issued = []

    def command():
        for _ in range(100):
            yield
        for value, wait in [(0, 130), (255, 130), (128, 100)]:
            yield dut.command.data.eq(value)
            yield dut.command.load.eq(1)
            yield
            yield dut.command.load.eq(0)
            for _ in range(wait):
                yield

    def mmcm():
        remaining     = 0
        direction     = None
        previous_psen = 0
        for tick in range(1500):
            psen = yield dut.psen
            assert not (psen and previous_psen), "PSEN must be a one-cycle pulse"
            previous_psen = psen
            if tick < 250:
                assert not psen, "Phase shift issued before the first tuning command"
            if remaining:
                assert not psen, "Overlapping MMCM phase shifts"
                assert (yield dut.psincdec) == direction
                remaining -= 1
            if psen:
                direction = yield dut.psincdec
                directions.append(direction)
                issued.append(tick)
                remaining = 23 # Slower than the requested maximum rate.
            yield dut.psdone.eq(remaining == 1)
            if tick > 1300:
                assert not psen, "Neutral command did not stop phase shifts"
            assert not (yield dut.fault)
            yield
    simulate(dut, {"wr": command(), "ps": mmcm()})
    assert 0 in directions and 1 in directions
    assert all(b - a >= 23 for a, b in zip(issued, issued[1:]))


def test_mmcm_timeout_stops_requests_and_reset_recovers():
    dut = WRMMCMBackend("ps", "wr", width=8, timeout_cycles=20)
    issued = []

    def command():
        for _ in range(10):
            yield
        yield dut.command.data.eq(0)
        yield dut.command.load.eq(1)
        yield
        yield dut.command.load.eq(0)
        for _ in range(200):
            yield
        yield dut.cd_wr.rst.eq(1)
        for _ in range(10):
            yield
        yield dut.cd_wr.rst.eq(0)

    def monitor():
        for tick in range(800):
            if (yield dut.psen):
                issued.append(tick)
            if 200 < tick < 500:
                assert (yield dut.fault)
            if tick > 650:
                assert not (yield dut.fault)
                assert not (yield dut.busy)
                assert not (yield dut.psen)
            yield
    simulate(dut, {"wr": command(), "ps": monitor()})
    assert len(issued) == 1


@pytest.mark.parametrize("code,div_n", [(1, 0), (64, 0), (128, 0), (192, 0), (255, 0), (64, 2), (192, 2)])
def test_mmcm_preserves_legacy_steady_state_rate_and_polarity(code, div_n):
    dut    = WRMMCMBackend("ps", "wr", width=8, div_n=div_n)
    issued = []
    cycles = 4096

    def command():
        for _ in range(10):
            yield
        yield dut.command.data.eq(code)
        yield dut.command.load.eq(1)
        yield
        yield dut.command.load.eq(0)

    def mmcm():
        pending = 0
        # Completion is faster than the highest nominal rate. Check steady
        # commands after CDC/startup, independently of the accumulator phase.
        for tick in range(200 + cycles):
            if pending:
                pending -= 1
            if (yield dut.psen):
                assert pending == 0
                pending = 8
                assert (yield dut.psincdec) == int(code < 128)
                if tick >= 200:
                    issued.append(tick)
            yield dut.psdone.eq(pending == 1)
            assert not (yield dut.fault)
            yield

    simulate(dut, {"wr": command(), "ps": mmcm()})
    # The original PSGen uses this transfer function. Endpoint code zero had
    # a magnitude-width bug and is covered by the corrected full-range tests.
    expected = cycles * abs(code - 128) / (1 << (8 + div_n + 3))
    assert abs(len(issued) - expected) <= 1


def test_legacy_psgen_keeps_running_without_completion_input():
    dut    = PSGen("ps", "wr", ctrl_size=8)
    issued = []

    def command():
        yield dut.ctrl_data.eq(192)
        yield dut.ctrl_load.eq(1)
        yield
        yield dut.ctrl_load.eq(0)

    def monitor():
        for tick in range(1024):
            if (yield dut.psen):
                assert (yield dut.psincdec) == 0
                issued.append(tick)
            yield

    simulate(dut, {"wr": command(), "ps": monitor()})
    assert len(issued) >= 30
    assert issued[-1] > 900


@pytest.mark.parametrize("code", [64, 192])
def test_mmcm_repeated_commands_preserve_fractional_phase(code):
    dut    = WRMMCMBackend("ps", "wr", width=8)
    issued = []
    cycles = 2048

    @passive
    def command():
        # Refresh the command more often than one requested phase step. A
        # constant servo output must have the same rate at every update rate.
        while True:
            yield dut.command.data.eq(code)
            yield dut.command.load.eq(1)
            yield
            yield dut.command.load.eq(0)
            for _ in range(7):
                yield

    def mmcm():
        pending = 0
        for tick in range(200 + cycles):
            if pending:
                pending -= 1
            if (yield dut.psen):
                assert pending == 0
                assert (yield dut.psincdec) == int(code < 128)
                pending = 12
                if tick >= 200:
                    issued.append(tick)
            yield dut.psdone.eq(pending == 1)
            assert not (yield dut.fault)
            yield

    simulate(dut, {"wr": command(), "ps": mmcm()})
    expected = cycles * abs(code - 128) / (1 << 11)
    assert abs(len(issued) - expected) <= 1
