#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from types import SimpleNamespace

import pytest

from migen import CEInserter, ClockDomain, Instance, Record, Signal, run_simulation

from litex_wr_nic.gateware.ad9516.core import AD9516PLL, AD9516_EXT_CONFIG
from litex_wr_nic.gateware.wr_clock import WRClockPresence
from litex_wr_nic.gateware.wr_core import WhiteRabbitCore


@pytest.mark.parametrize("stop_after", [130, 170])
def test_reference_detection_and_recovery(stop_after):
    # Freezing the input prescaler models a clock stopping at either phase.
    dut = CEInserter(["clk10m_in"])(WRClockPresence(timeout=64))

    def check():
        for _ in range(80):
            assert (yield dut.present) == 0
            yield
        yield dut.ce_clk10m_in.eq(1)
        for _ in range(stop_after):
            yield
        assert (yield dut.present) == 1
        yield dut.ce_clk10m_in.eq(0)
        for _ in range(80):
            yield
        assert (yield dut.present) == 0
        yield dut.ce_clk10m_in.eq(1)
        for _ in range(150):
            yield
        assert (yield dut.present) == 1

    run_simulation(dut, check(), clocks={"sys": 10, "clk10m_in": 50})


def test_ad9516_lock_requires_configuration_and_no_reset(monkeypatch):
    monkeypatch.setattr(AD9516PLL, "add_sources", lambda *args: None)
    pads = Record([(name, 1) for name in
        ("lock", "reset_n", "stat", "refsel", "sync_n", "cs_n", "sck", "sdi", "sdo")])
    dut = AD9516PLL(None, pads, AD9516_EXT_CONFIG, "sync_in")
    dut.cd_sys = ClockDomain("sys")
    fragment = dut.get_fragment()
    instance = next(s for s in fragment.specials if isinstance(s, Instance))
    fragment.specials.remove(instance)
    reset_n = Signal()
    fragment.comb += [reset_n.eq(next(item.expr for item in instance.items if item.name == "rst_n_i"))]

    def check():
        yield pads.lock.eq(1)
        for _ in range(5):
            yield
        assert (yield dut.locked) == 0
        yield dut._done.status.eq(1)
        yield
        assert (yield dut.locked) == 1
        for reset in (dut.reset, dut._rst.storage):
            yield reset.eq(1)
            yield
            assert (yield reset_n) == 0
            assert (yield dut._locked.status) == 0
            yield reset.eq(0)
            yield
        yield pads.lock.eq(0)
        for _ in range(5):
            yield
        assert (yield dut.locked) == 0

    run_simulation(fragment, check())


@pytest.mark.parametrize("external", [False, True])
def test_spec_selects_external_multiplier(monkeypatch, external):
    from spec_a7_wr_nic import BaseSoC

    monkeypatch.setattr(WhiteRabbitCore, "add_sources", staticmethod(lambda platform: None))
    monkeypatch.setattr(AD9516PLL, "add_sources", lambda *args: None)
    soc = BaseSoC(with_pcie=False, with_sync_in_pll=external, white_rabbit_cpu_firmware="unused.bram")
    fragment = soc.get_fragment()
    instance = next(s for s in fragment.specials
        if isinstance(s, Instance) and s.of == "xwrc_board_litex_wr_nic_wrapper")
    parameters = {item.name: item.value for item in instance.items if isinstance(item, Instance.Parameter)}
    assert parameters["g_with_external_clock_input"].value == 1
    assert parameters["g_use_external_pll"].value == int(external)
    assert hasattr(soc, "sync_in_pll") == external
    if external:
        ports = {item.name: item.expr for item in instance.items if isinstance(item, (Instance.Input, Instance.Output))}
        assert ports["clk_ext_locked_i"] is soc.wr_core.ext_clk_locked
        assert ports["clk_ext_stopped_i"] is soc.wr_core.ext_clk_stopped
        assert ports["clk_ext_rst_o"] is soc.wr_core.ext_clk_reset


def test_external_multiplier_requires_external_reference():
    with pytest.raises(ValueError, match="external clock input"):
        WhiteRabbitCore(SimpleNamespace(), cpu_firmware="unused.bram", with_ext_clk=False, with_ext_pll=True)
