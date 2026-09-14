#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from pathlib import Path
from types import SimpleNamespace

import pytest

from litex_wr_nic.rf_pll import LMX2572, LMX2572Config


CONFIG = Path(__file__).with_name("rf_pll_25m_to_100m.txt")
TCS = Path(__file__).with_name("LMX2572_125_122.88MHz.tcs")


def make_bus(status=3, with_sync=True):
    writes = []
    regs = SimpleNamespace()
    for suffix in ("status", "control", "mosi", "cs", "loopback", "sync"):
        if suffix == "sync" and not with_sync:
            continue
        setattr(regs, "rf_out_pll_" + suffix, SimpleNamespace(
            read=lambda: status,
            write=lambda value, suffix=suffix: writes.append((suffix, value))))
    return SimpleNamespace(regs=regs), writes


def test_native_configuration_and_programming_sequence():
    config = LMX2572Config.from_file(CONFIG)
    assert config.frequencies() == dict(reference=25e6, pfd=25e6, vco=6.4e9, out_a=100e6, out_b=100e6)
    writes = list(config.writes())
    assert writes[0] == (0, 0x221e)
    assert writes[-1] == (0, 0x201c)
    assert [addr for addr, value in writes[1:]] == [addr for addr in range(125, -1, -1) if addr not in range(107, 114)]


def test_tcs_reference_mismatch_is_rejected_before_any_bus_write():
    config = LMX2572Config.from_file(TCS)
    assert config.frequencies(125e6)["out_a"] == pytest.approx(122.88e6, abs=0.001)
    bus, writes = make_bus()
    with pytest.raises(ValueError, match="expects a 125 MHz reference"):
        LMX2572(bus).load(config)
    assert writes == []


def test_missing_metadata_still_rejects_wrong_reference_dividers():
    config = LMX2572Config.from_file(TCS)
    config.reference_frequency = None
    with pytest.raises(ValueError, match="PFD frequency"):
        config.frequencies()


@pytest.mark.parametrize("text", [
    "R0 0x00201c\nR1 garbage\n",
    "R1 0x00201c\n",
    "R0 0x00201c\nR0 0x00201c\n",
    "R128 0x800000\n",
    "[MODES]\nVALUE00=garbage\n",
    "[MODES]\nVALUE00=8220\nVALUE01=8220\n",
    "[SETUP]\nPART=LMX2594\n[MODES]\nVALUE00=8220\n",
])
def test_malformed_file_is_rejected_without_partial_programming(tmp_path, text):
    path = tmp_path / ("config.tcs" if text.startswith("[") else "config.txt")
    path.write_text(text)
    bus, writes = make_bus()
    with pytest.raises(ValueError):
        LMX2572(bus).load(path)
    assert writes == []


@pytest.mark.parametrize("address,value", [(12, 0x5000), (11, 0xb008), (10, 0x1078),
    (44, 0x1f27), (36, 10), (39, 0), (75, 0x0880), (0, 0xa01c), (0, 0x2014), (114, 0x7c00)])
def test_invalid_frequency_configurations_are_rejected(address, value):
    config = LMX2572Config.from_file(CONFIG)
    config.registers[address] = value
    with pytest.raises(ValueError):
        config.frequencies()


def test_load_resets_spi_mode_and_sends_complete_24_bit_writes(monkeypatch):
    delays = []
    monkeypatch.setattr("litex_wr_nic.rf_pll.time.sleep", delays.append)
    bus, writes = make_bus()
    config = LMX2572Config.from_file(CONFIG)
    LMX2572(bus).load(config)
    assert delays == [0.0005]
    assert writes[0] == ("sync", 0)
    assert writes[1:] == [item for address, value in config.writes() for item in
        (("cs", 1), ("loopback", 0), ("mosi", (address << 16) | value), ("control", 0x1801))]


def test_invalid_late_register_is_rejected_before_reset():
    config = LMX2572Config.from_file(CONFIG)
    config.registers[7] = 0x10000
    bus, writes = make_bus()
    with pytest.raises(ValueError, match="16-bit values"):
        LMX2572(bus).load(config)
    assert writes == []


@pytest.mark.parametrize("address,value", [(-1, 0), (126, 0), (110, 0), (0, -1), (0, 65536)])
def test_raw_writes_do_not_truncate_invalid_values(address, value):
    bus, writes = make_bus()
    with pytest.raises(ValueError):
        LMX2572(bus).write_reg(address, value)
    assert writes == []


def test_busy_timeout_sends_no_command():
    bus, writes = make_bus(status=2)
    with pytest.raises(TimeoutError):
        LMX2572(bus, timeout=0.001).write_reg(36, 256)
    assert writes == []


def test_transfer_failure_stops_configuration():
    bus, writes = make_bus(status=7)
    with pytest.raises(RuntimeError, match="rejected"):
        LMX2572(bus).load(CONFIG)
    assert [value for suffix, value in writes if suffix == "control"] == [0x1801]


def test_legacy_bitstream_load_and_missing_sync():
    bus, writes = make_bus(with_sync=False)
    LMX2572(bus).load(CONFIG)
    with pytest.raises(RuntimeError, match="does not expose"):
        LMX2572(bus).toggle_sync()


def test_sync_returns_low_even_on_interruption(monkeypatch):
    def interrupt(delay):
        raise KeyboardInterrupt()
    monkeypatch.setattr("litex_wr_nic.rf_pll.time.sleep", interrupt)
    bus, writes = make_bus()
    with pytest.raises(KeyboardInterrupt):
        LMX2572(bus).toggle_sync()
    assert writes == [("sync", 0), ("sync", 1), ("sync", 0)]
