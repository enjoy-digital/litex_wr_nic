"""An asynchronous prompt redraw must not acknowledge a command prematurely."""

import importlib.util
from pathlib import Path


def test_command_waits_for_its_echo_and_completion_prompt(monkeypatch):
    path = Path(__file__).resolve().parents[1] / "tools/wr_console.py"
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


def test_diagnostic_between_echo_characters_does_not_hide_completion(monkeypatch):
    path = Path(__file__).resolve().parents[1] / "tools/wr_console.py"
    spec = importlib.util.spec_from_file_location("wr_console_interleaving_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    console = module.Console.__new__(module.Console)
    # Captured during a real Acorn PCS interruption. An old prompt still
    # cannot acknowledge the command before the remaining echo arrives.
    responses = iter([
        "vdiag-fsm-1-wr0: 000000127.819: LEAVE slave (next:   3)\n\nwrc# ",
        "vdiag-fsm-1-wr0: 000000127.819: LEAVE slave (next:   3)\n\n"
        "er\nWR Core build: v8.0-126-gbaf77496-dirty\nwrc# ",
    ])
    monkeypatch.setattr(console, "text", lambda start: next(responses))
    monkeypatch.setattr(console, "record", lambda *args: None)
    monkeypatch.setattr(module.time, "sleep", lambda seconds: None)
    response = console.prompt(0, command="ver")
    assert "WR Core build:" in response
    assert "diag-fsm-1-wr0:" in response
