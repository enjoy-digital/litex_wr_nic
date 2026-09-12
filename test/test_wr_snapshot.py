"""Acquisition snapshots must be retained without qualifying invalid time."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

path = Path(__file__).resolve().parents[1] / 'tools/wr_status.py'
spec = importlib.util.spec_from_file_location('wr_status_test', path)
status = importlib.util.module_from_spec(spec)
spec.loader.exec_module(status)


class SnapshotBus:
    mems = SimpleNamespace(wr_wb_slave=SimpleNamespace(base=0))

    def __init__(self, nanoseconds):
        self.words = [0] * 25
        self.words[0:5] = [2, 0x101, 0x401, 3, 9]
        self.words[9:11] = [123, nanoseconds]
        self.writes = []

    def read(self, address, length=None):
        if length:
            return list(self.words[:length])
        return self.words[(address - 0x900) // 4]

    def write(self, address, value):
        self.writes.append((address, value))


@pytest.mark.parametrize('nanoseconds', [999999999, 1000000000, ((1 << 28) - 100) * 16])
def test_snapshot_marks_wrapped_counter_time_invalid(nanoseconds):
    bus = SnapshotBus(nanoseconds)
    result = status.diagnostic_snapshot(bus)
    assert result['raw'][10] == nanoseconds
    assert result['time_valid'] == (nanoseconds < 1000000000)
    assert result['tai_ns'] == (123000000000 + nanoseconds if nanoseconds < 1000000000 else None)
    assert bus.writes[-1] == (0x904, 0)


def test_invalid_time_cannot_start_or_pass_a_tracking_check():
    common = dict(received=100, extension='IDLE', detection='EXT_ON', pll_locked=True,
                  frequency_locked=True, servo='TRACK_PHASE')
    master = dict(common, role='MASTER', mac='a', peer='b')
    slave = dict(common, role='SLAVE', mac='b', peer='a')
    md = dict(ptp_state=6, link=True, locked=True, tai_ns=123000000000)
    sd = dict(md, ptp_state=9, servo_valid=True, servo_state=4, tai_ns=None)
    assert 'slave: timestamp is not normalized' in status.qualification_errors(master, slave, md, sd, 101)
    sd['tai_ns'] = 123500000000
    assert status.qualification_errors(master, slave, md, sd, 101) == []
