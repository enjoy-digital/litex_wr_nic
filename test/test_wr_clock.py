#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import shutil
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

from migen import *
from migen.fhdl.structure import _Assign

from litex.gen import LiteXModule

from litex_wr_nic.gateware.ps_gen import PSGen
from litex_wr_nic.gateware.wr_clock import WRTuningCalibration, WRTuningCDC, WRDACBackend, WRMMCMBackend
from litex_wr_nic.gateware.wr_core import WhiteRabbitCore
from litex_wr_nic.gateware.ad5683r.core import AD5683RDAC

# Constants ----------------------------------------------------------------------------------------

ROOT = Path(__file__).resolve().parents[1]

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


def test_default_dac_path_preserves_registered_wr_commands():
    dut = LiteXModule()
    dut.tuning = WRDACBackend(cd="wr")
    pads = Record([(name, 1) for name in ("ldac_n", "sync_n", "sclk", "sdi")], name="pads")
    dut.dac = AD5683RDAC(SimpleNamespace(add_source=lambda path: None), pads,
        load=dut.tuning.load, value=dut.tuning.value, clk_domain="wr")
    dut.cd_sys = ClockDomain("sys")
    dut.cd_wr  = ClockDomain("wr")

    # Before the backend was introduced, the DAC driver registered WR's code
    # and load once in wr_sys. Preserve the same stream with default settings.
    expected_value = Signal(16)
    expected_load  = Signal()
    dut.sync.wr += [
        expected_value.eq(dut.tuning.command.data),
        expected_load.eq(dut.tuning.command.load),
    ]
    fragment = dut.get_fragment()
    instance = next(
        s for s in fragment.specials
        if isinstance(s, Instance) and s.of == "serial_dac_arb"
    )
    inputs = {item.name: item.expr for item in instance.items if isinstance(item, Instance.Input)}
    fragment.specials.remove(instance)

    def check():
        # Include boundaries, alternating bits and repeated/bursty load pulses.
        for index, value in enumerate([0, 1, 32767, 32768, 32769, 65535, 0xaaaa, 0x5555]*4):
            yield dut.tuning.command.data.eq(value)
            yield dut.tuning.command.load.eq(index % 3 != 0)
            yield
            assert (yield inputs["load_i"]) == (yield expected_load)
            if (yield expected_load):
                assert (yield inputs["val_i"]) == (yield expected_value)
        yield dut.tuning.command.load.eq(0)
        yield
        yield
        assert not (yield inputs["load_i"])

    clocks = {"sys": 10, "wr": 16, dut.dac.host_cdc.input_cd: 10, dut.dac.host_cdc.output_cd: 16}
    run_simulation(fragment, {"wr": check()}, clocks=clocks)


@pytest.mark.parametrize("gain", [None, 1, 2])
def test_dac_serial_words_preserve_default_x2_gain(tmp_path, gain):
    if shutil.which("ghdl") is None:
        pytest.skip("GHDL is required for the DAC serial regression")
    pads = Record([(name, 1) for name in ("ldac_n", "sync_n", "sclk", "sdi")], name="pads")
    kwargs = {} if gain is None else dict(gain=gain)
    dut = AD5683RDAC(SimpleNamespace(add_source=lambda path: None), pads,
        load=Signal(), value=Signal(16), **kwargs)
    instance = next(
        s for s in dut.get_fragment().specials
        if isinstance(s, Instance) and s.of == "serial_dac_arb"
    )
    params = {item.name: item.value for item in instance.items if isinstance(item, Instance.Parameter)}
    x2 = params["g_enable_x2_gain"].value
    assert x2 == int(gain != 1)
    driver = ROOT / "litex_wr_nic/gateware/ad5683r"

    def run(*args):
        result = subprocess.run(["ghdl", args[0], "--std=08", *args[1:]],
            cwd            = tmp_path,
            capture_output = True,
            text           = True,
            timeout        = 30,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        return result.stdout + result.stderr

    run("-a", str(driver / "serial_dac.vhd"), str(driver / "serial_dac_arb.vhd"),
        str(ROOT / "test/hdl/wr_dac_tb.vhd"))
    run("-e", "wr_dac_tb")
    output = run("-r", "wr_dac_tb", f"-gx2_gain={'true' if x2 else 'false'}",
        "--assert-level=error", "--stop-time=100us")
    assert "DAC serial checks passed" in output


def test_spec_a7_preserves_effective_refclk_and_dmtd_gain(monkeypatch):
    from spec_a7_wr_nic import BaseSoC

    monkeypatch.setattr(WhiteRabbitCore, "add_sources", staticmethod(lambda platform: None))
    soc = BaseSoC(with_pcie=False, with_rf_out=False, with_sync_in_pll=False)
    for dac in (soc.refclk_dac, soc.dmtd_dac):
        instance = next(
            s for s in dac.get_fragment().specials
            if isinstance(s, Instance) and s.of == "serial_dac_arb"
        )
        params = {item.name: item.value for item in instance.items if isinstance(item, Instance.Parameter)}
        assert params["g_enable_x2_gain"].value == 1


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
