#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from types import SimpleNamespace

import pytest
from migen import ClockDomain, ClockSignal, Instance, Record, Signal, run_simulation
from migen.fhdl.structure import _Assign
from migen.sim import passive

from litex_wr_nic.gateware.ad5683r.core import AD5683RDAC


def make_dut():
    pads = Record([(name, 1) for name in ("ldac_n", "sync_n", "sclk", "sdi")])
    load, value = Signal(), Signal(16)
    dut = AD5683RDAC(SimpleNamespace(add_source=lambda path: None), pads, load, value, clk_domain="wr")
    dut.cd_sys = ClockDomain("sys")
    dut.cd_wr = ClockDomain("wr")
    fragment = dut.get_fragment()
    driver, = [s for s in fragment.specials if isinstance(s, Instance)]
    ports = {item.name: item.expr for item in driver.items if isinstance(item, Instance.Input)}
    # Observe the actual serial arbiter inputs; its physical SPI is tested in
    # test_ad5683r.py. This simulation exercises the complete CSR/CDC wrapper.
    fragment.specials.remove(driver)
    return dut, fragment, load, value, ports


def simulate(fragment, generators, clocks):
    clocks = dict(clocks)
    for statement in fragment.comb:
        if isinstance(statement, _Assign) and isinstance(statement.r, ClockSignal):
            if statement.r.cd in clocks:
                for domain in fragment.clock_domains:
                    if statement.l is domain.clk:
                        clocks[domain.name] = clocks[statement.r.cd]
    run_simulation(fragment, generators, clocks=clocks)


def wait_ready(dut):
    for _ in range(500):
        if (yield dut._status.fields.ready):
            return
        yield
    raise AssertionError("DAC command queue did not become ready")


def test_current_latches_only_commands_delivered_to_driver():
    dut, fragment, load, value, ports = make_dut()
    delivered, observed = [], []

    def wr():
        for tick in range(250):
            yield value.eq((tick * 997) & 0xffff)
            yield load.eq(tick in (20, 70, 120))
            yield

    @passive
    def driver():
        while True:
            if (yield ports["rst_n_i"]) and (yield ports["load_i"]):
                delivered.append((yield ports["val_i"]))
            yield

    def host():
        for _ in range(600):
            observed.append((yield dut._current.status))
            yield

    simulate(fragment, {"sys": host(), "wr": [wr(), driver()]}, {"sys": 10, "wr": 17})
    assert delivered == [20 * 997, (70 * 997) & 0xffff, (120 * 997) & 0xffff]
    assert set(observed) == {0, *delivered}
    assert observed[-1] == delivered[-1]


@pytest.mark.parametrize("clocks", [{"sys": 10, "wr": 17}, {"sys": 19, "wr": 6}])
def test_host_loads_are_ordered_pulses_with_atomic_values(clocks):
    dut, fragment, load, value, ports = make_dut()
    delivered = []

    def host():
        yield from wait_ready(dut)
        yield from dut._force.write(1)
        for code in (0x00ff, 0xff00, 0xa55a):
            yield from dut._value.write(code)
            yield from wait_ready(dut)
            yield from dut._load.write(1)
            # The old 1/0 write sequence remains accepted, without a second
            # load; changing value afterward must not change the queued word.
            yield from dut._load.write(0)
            yield from dut._value.write(code ^ 0xffff)
        for _ in range(400):
            yield
        assert (yield dut._status.fields.forced) == 1
        assert (yield dut._status.fields.overflow) == 0
        assert (yield dut._current.status) == 0xa55a

    @passive
    def driver():
        previous = 0
        while True:
            yield value.eq(0x1234)
            active = yield ports["load_i"]
            assert not (previous and active), "Host loads must have a low cycle between pulses"
            previous = active
            if active:
                delivered.append((yield ports["val_i"]))
            yield

    simulate(fragment, {"sys": host(), "wr": driver()}, clocks)
    assert delivered == [0x00ff, 0xff00, 0xa55a]


def test_mode_changes_do_not_load_and_release_restores_wr_commands():
    dut, fragment, load, value, ports = make_dut()
    delivered = []
    resume = Signal()

    def host():
        yield from wait_ready(dut)
        yield from dut._value.write(0xdead)
        yield from dut._force.write(1)
        for _ in range(100):
            yield
        assert (yield dut._current.status) == 0
        yield from dut._value.write(0xabcd)
        yield from dut._load.write(1)
        yield from wait_ready(dut)
        yield from dut._force.write(0)
        for _ in range(150):
            yield
        assert (yield dut._status.fields.forced) == 0
        assert (yield dut._current.status) == 0xabcd
        yield resume.eq(1)
        for _ in range(200):
            yield
        assert (yield dut._current.status) == 0x5678

    @passive
    def wr():
        sent = False
        while True:
            yield load.eq(0)
            yield value.eq(0x5678)
            if (yield resume) and not sent:
                yield load.eq(1)
                sent = True
            if (yield ports["load_i"]):
                delivered.append((yield ports["val_i"]))
            yield

    simulate(fragment, {"sys": host(), "wr": wr()}, {"sys": 10, "wr": 17})
    assert delivered == [0xabcd, 0x5678]


def test_overflow_is_reported_and_wr_reset_clears_queued_commands():
    dut, fragment, load, value, ports = make_dut()
    delivered_after_reset = []
    after_reset = Signal()

    def host():
        yield from wait_ready(dut)
        yield from dut._force.write(1)
        for code in range(20):
            yield from dut._value.write(code)
            yield from dut._load.write(1)
        for _ in range(10):
            yield
        assert (yield dut._status.fields.overflow) == 1
        yield dut.cd_wr.rst.eq(1)
        for _ in range(120):
            yield
        yield dut.cd_wr.rst.eq(0)
        yield after_reset.eq(1)
        for _ in range(600):
            yield
        assert (yield dut._status.fields.overflow) == 0
        assert (yield dut._status.fields.forced) == 0
        assert (yield dut._current.status) == 0

    @passive
    def driver():
        while True:
            if (yield after_reset) and (yield ports["load_i"]):
                delivered_after_reset.append((yield ports["val_i"]))
            yield

    simulate(fragment, {"sys": host(), "wr": driver()}, {"sys": 6, "wr": 50})
    assert delivered_after_reset == []
