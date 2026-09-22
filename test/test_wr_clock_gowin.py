#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import pytest

from migen import *

from litex_wr_nic.gateware.ms5351 import (
    MS5351_CODE_NOMINAL,
    MS5351PLLTuner,
    ms5351_pll_registers,
)
from litex_wr_nic.gateware.wr_clock import WRGowinPLLBackend, WRMS5351Backend

# Helpers ------------------------------------------------------------------------------------------

def _command(backend, value):
    yield backend.command.data.eq(value)
    yield backend.command.load.eq(1)
    yield
    yield backend.command.load.eq(0)
    yield


class I2CMonitor:
    """Decode the tuner's open-drain intents into transactions of bytes."""
    def __init__(self, tuner):
        self.tuner        = tuner
        self.transactions = []
        self._scl = 1
        self._sda = 1
        self._bits = []

    def sample(self):
        scl = (yield self.tuner.scl_o)
        sda = (yield self.tuner.sda_o)
        if scl and self._scl:
            if self._sda and not sda:
                self.transactions.append([])
                self._bits = []
            elif sda and not self._sda:
                self._bits = []
        if scl and not self._scl:
            self._bits.append(sda)
            if len(self._bits) == 8:
                self.transactions[-1].append(int("".join(map(str, self._bits)), 2))
            elif len(self._bits) == 9:
                self._bits = []
        self._scl = scl
        self._sda = sda

# MS5351 Register Model ----------------------------------------------------------------------------

def test_pll_registers_span_one_integer_step():
    nominal = ms5351_pll_registers(36, MS5351_CODE_NOMINAL)
    assert nominal == bytes([0xff, 0xff, 0x00, 0x10, 0x00, 0xf0, 0x00, 0x00])
    below = ms5351_pll_registers(36, MS5351_CODE_NOMINAL - 1)
    assert below[2:5] == bytes([0x00, 0x0f, 0xff]) and below[5:] == bytes([0xff, 0xff, 0xff])
    assert ms5351_pll_registers(32, 0)[2:5] == bytes([0x00, 0x0d, 0xff])
    with pytest.raises(ValueError):
        ms5351_pll_registers(36, 1 << 21)

# Gowin PLL Backend --------------------------------------------------------------------------------

@pytest.mark.parametrize("code,direction", [(32768 + 4096, 1), (32768 - 4096, 0)])
def test_gowin_backend_steps_at_the_commanded_rate(code, direction):
    backend = WRGowinPLLBackend(cd="sys", div_n=1)

    def stimulus():
        yield from _command(backend, code)
        for _ in range(300):
            yield
        steps = 0
        for _ in range(2048):
            steps += (yield backend.phase_step)
            if (yield backend.phase_step):
                assert (yield backend.phase_dir) == direction
            yield
        # Steady state: 4096 / 2**20 steps per cycle, one every 256 cycles.
        assert steps == 8
        assert (yield backend.steps) == steps + 1
        yield from _command(backend, 32768)
        for _ in range(512):
            assert (yield backend.phase_step) == 0
            yield

    run_simulation(backend, stimulus())


def test_gowin_backend_reversal_restarts_the_accumulator():
    backend = WRGowinPLLBackend(cd="sys", div_n=1)

    def stimulus():
        yield from _command(backend, 32768 + 8192)
        for _ in range(100):
            yield
        yield from _command(backend, 32768 - 8192)
        steps = 0
        for _ in range(200):
            steps += (yield backend.phase_step)
            yield
        # One step per 128 cycles, and the first one needs a full accumulation:
        # the 100 cycles of progress in the previous direction are discarded.
        assert steps == 1

    run_simulation(backend, stimulus())

# MS5351 Backend -----------------------------------------------------------------------------------

def test_ms5351_backend_writes_changed_registers():
    backend = WRMS5351Backend(1e6, center=MS5351_CODE_NOMINAL, i2c_freq=250e3,
        select_cycles=4, release_cycles=4, multiplier=36)
    monitor = I2CMonitor(backend.tuner)

    def stimulus():
        yield backend.tuner.sda_i.eq(0) # Every byte is acknowledged.
        # Neutral command: the first update rewrites all eight registers.
        for _ in range(600):
            yield from monitor.sample()
            yield
        assert (yield backend.tuner.valid) == 1
        assert monitor.transactions == [[0xc0, 26, *ms5351_pll_registers(36, MS5351_CODE_NOMINAL)]]
        # A small change writes the three P2 bytes from register 31.
        yield from _command(backend, 32768 + 5)
        for _ in range(400):
            yield from monitor.sample()
            yield
        registers = ms5351_pll_registers(36, MS5351_CODE_NOMINAL + 5)
        assert monitor.transactions[-1] == [0xc0, 31, *registers[5:]]
        assert (yield backend.tuner.current) == MS5351_CODE_NOMINAL + 5
        # Crossing the integer step rewrites P1 as well, from register 28.
        yield from _command(backend, 32768 - 5)
        for _ in range(500):
            yield from monitor.sample()
            yield
        registers = ms5351_pll_registers(36, MS5351_CODE_NOMINAL - 5)
        assert monitor.transactions[-1] == [0xc0, 28, *registers[2:]]
        assert (yield backend.tuner.updates) == 3
        assert (yield backend.tuner.errors) == 0

    run_simulation(backend, stimulus())


def test_ms5351_backend_scales_and_saturates_commands():
    backend = WRMS5351Backend(1e6, center=100, shift=4)

    def stimulus():
        yield backend._control.storage.eq(0)
        yield from _command(backend, 32768 + 16)
        assert (yield backend.tuner.code) == 100 + (16 << 4)
        yield from _command(backend, 0)
        assert (yield backend.tuner.code) == 0
        yield backend._center.storage.eq((1 << 21) - 10)
        yield from _command(backend, 65535)
        yield
        assert (yield backend.tuner.code) == (1 << 21) - 1

    run_simulation(backend, stimulus())


def test_ms5351_tuner_reports_missing_acknowledge():
    tuner = MS5351PLLTuner(1e6, i2c_freq=250e3, select_cycles=4, release_cycles=4)

    def stimulus():
        yield tuner.sda_i.eq(1) # Nothing acknowledges.
        yield tuner.enable.eq(1)
        for _ in range(200):
            yield
        assert (yield tuner.errors) >= 1
        assert (yield tuner.valid) == 0
        assert (yield tuner.updates) == 0

    run_simulation(tuner, stimulus())
