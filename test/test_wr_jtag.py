"""Persistent ownership must survive an orphan without signaling reused PIDs."""
import importlib.util
import json
from pathlib import Path
from unittest.mock import Mock

import pytest

spec = importlib.util.spec_from_file_location('wr_jtag', Path(__file__).resolve().parents[1] / 'tools/wr_jtag.py')
jtag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(jtag)


def record(tmp_path, boot=None):
    path = tmp_path / 'owner.json'
    jtag.write_record(path, dict(
        boot_id=boot or Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        group=9001, members=[dict(pid=9001, started='100'), dict(pid=9002, started='101')]))
    return path


def test_old_boot_does_not_signal_processes(tmp_path, monkeypatch):
    kill = Mock()
    monkeypatch.setattr(jtag.os, 'killpg', kill)
    path = record(tmp_path, 'old-boot')
    jtag.stop(path)
    kill.assert_not_called()
    assert not path.exists()


def test_pid_reuse_refuses_cleanup(tmp_path, monkeypatch):
    kill = Mock()
    monkeypatch.setattr(jtag.os, 'killpg', kill)
    monkeypatch.setattr(jtag, 'members', lambda group: [dict(pid=9001, started='200')])
    path = record(tmp_path)
    with pytest.raises(RuntimeError, match='unverified'):
        jtag.stop(path)
    kill.assert_not_called()
    assert path.exists()


def test_persisted_child_allows_cleanup_after_leader_dies(tmp_path, monkeypatch):
    live = [dict(pid=9002, started='101')]
    signals = []
    def kill(group, signal):
        signals.append((group, signal))
        live.clear()
    monkeypatch.setattr(jtag.os, 'killpg', kill)
    monkeypatch.setattr(jtag, 'members', lambda group: live[:])
    path = record(tmp_path)
    jtag.stop(path)
    assert signals == [(9001, jtag.signal.SIGTERM)]
    assert not path.exists()


def test_config_paths_and_unique_ports(tmp_path):
    config = dict(state_dir='state', boards=dict(acorn=dict(csr='csr.csv',
        uart='/dev/serial/by-path/board', jtag_config='jtag.cfg', jtag_port=1235, stream_port=20001)))
    path = tmp_path / 'bench.json'
    path.write_text(json.dumps(config))
    loaded = jtag.load_config(path)
    assert loaded['boards']['acorn']['csr'] == str(tmp_path / 'csr.csv')
    assert loaded['boards']['acorn']['uart'] == '/dev/serial/by-path/board'
    config['boards']['acorn']['stream_port'] = 1235
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match='distinct'):
        jtag.load_config(path)
