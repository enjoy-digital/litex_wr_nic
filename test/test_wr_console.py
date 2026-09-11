"""An asynchronous prompt redraw must not acknowledge a command prematurely."""

import importlib.util
from pathlib import Path


def test_command_waits_for_its_echo_and_completion_prompt(monkeypatch):
    path = Path(__file__).resolve().parents[1] / "tools/wr_link.py"
    spec = importlib.util.spec_from_file_location("wr_console_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    console = module.Console.__new__(module.Console)
    responses = iter(
        ["asynchronous log\nwrc# ", "asynchronous log\nwrc# ptp\nrunning; e2e master\nwrc# "]
    )
    monkeypatch.setattr(console, "text", lambda start: next(responses))
    monkeypatch.setattr(console, "record", lambda *args: None)
    monkeypatch.setattr(module.time, "sleep", lambda seconds: None)
    response = console.prompt(0, command="ptp")
    assert "running; e2e master" in response
