#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from types import SimpleNamespace

import pytest

from migen import *
from migen.fhdl.structure import _Assign

from litex_wr_nic.gateware.ad5683r.core import AD5683RDAC
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


def test_dac_host_command_is_coherent_and_load_is_a_pulse():
    pads = Record([(name, 1) for name in ("ldac_n", "sync_n", "sclk", "sdi")], name="pads")
    dut = AD5683RDAC(SimpleNamespace(add_source=lambda path: None), pads,
        load=Signal(), value=Signal(16, reset=0x1234), gain=1, clk_domain="wr")
    dut.cd_sys = ClockDomain("sys")
    dut.cd_wr  = ClockDomain("wr")
    fragment = dut.get_fragment()
    instance = next(s for s in fragment.specials if isinstance(s, Instance) and s.of == "serial_dac_arb")
    params = {item.name: item.value for item in instance.items if isinstance(item, Instance.Parameter)}
    assert params["g_enable_x2_gain"].value == 0
    signals = {item.name: item.expr for item in instance.items if isinstance(item, Instance.Input)}
    fragment.specials.remove(instance)
    loaded = []

    def host():
        yield dut._force.storage.eq(1)
        yield dut._force.re.eq(1)
        yield
        yield dut._force.re.eq(0)
        for _ in range(30):
            yield
        yield dut._value.storage.eq(0xabcd)
        yield dut._load.storage.eq(1)
        yield dut._load.re.eq(1)
        yield
        yield dut._load.re.eq(0)
        for _ in range(80):
            yield
        # A level left high in the storage register must not hold load high.
        yield dut._force.storage.eq(0)
        yield dut._force.re.eq(1)
        yield
        yield dut._force.re.eq(0)
        for _ in range(50):
            yield
        assert (yield signals["val_i"]) == 0x1234

    def driver():
        for _ in range(120):
            if (yield signals["load_i"]):
                loaded.append((yield signals["val_i"]))
            yield
    clocks = {"sys": 10, "wr": 16, dut.host_cdc.input_cd: 10, dut.host_cdc.output_cd: 16}
    run_simulation(fragment, {"sys": host(), "wr": driver()}, clocks=clocks)
    assert loaded == [0xabcd]
