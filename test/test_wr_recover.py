"""Ensure a failed recovery probe restores the link and preserves a failure record."""

import importlib.util
import json
from pathlib import Path

import pytest


@pytest.fixture
def recovery(monkeypatch):
    tools = Path(__file__).resolve().parents[1] / "tools"
    monkeypatch.syspath_prepend(str(tools))
    spec = importlib.util.spec_from_file_location("wr_recover_test", tools / "wr_recover.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("restore_fails", [False, True])
def test_capture_failure_restores_mcr_and_closes_all_connections(
    recovery, monkeypatch, tmp_path, restore_fails
):
    class Bus:
        mems = type("Memories", (), {"wr_wb_slave": type("Memory", (), {"base": 0x20000000})})

        def __init__(self):
            self.writes = []
            self.closed = False

        def read(self, address):
            return 0x80001140

        def write(self, address, value):
            self.writes.append((address, value))
            if restore_fails and address == 0x10012C and value == 0x80001140:
                raise RuntimeError("restore failed")

        def close(self):
            self.closed = True

    consoles = []

    class Console:
        def __init__(self, *args):
            self.closed = False
            consoles.append(self)

        def connect(self):
            pass

        def command(self, command):
            return "wrc#"

        def close(self):
            self.closed = True

    class Board:
        config = {"uart": "fake"}

        def __init__(self, name):
            self.name = name
            self.bus = Bus()

        def client(self):
            return self.bus

    def fail_snapshot(bus):
        raise RuntimeError("capture failed")

    class DebugContext:
        def __init__(self, bus):
            self.bus = bus

        def __enter__(self):
            return self.bus

        def __exit__(self, *args):
            pass

    monkeypatch.setattr(recovery, "Console", Console)
    monkeypatch.setattr(recovery, "diagnostic_snapshot", fail_snapshot)
    monkeypatch.setattr(recovery, "URVDebug", DebugContext)
    slave, master = Board("slave"), Board("master")
    with pytest.raises(RuntimeError, match="restore failed" if restore_fails else "capture failed"):
        recovery.interrupt_link(slave, master, tmp_path, seconds=0)
    assert (0x10012C, 0x80001940) in slave.bus.writes
    assert slave.bus.writes[-1] == (0x10012C, 0x80001140)
    assert slave.bus.closed and master.bus.closed and all(c.closed for c in consoles)
    record = json.loads((tmp_path / "interruption.json").read_text())
    assert not record["passed"]
    assert record["error"] == ("restore failed" if restore_fails else "capture failed")
