"""Reject PTP fallback, stale monitoring and false servo-valid qualification."""

from copy import deepcopy
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("wr_status", ROOT / "tools/wr_status.py")
status = importlib.util.module_from_spec(spec)
spec.loader.exec_module(status)


def captured_frame(board, chunk):
    raw = (ROOT / "test/fixtures" / (board + "-wr-tracking.uart")).read_bytes()
    screen = status.WRScreen()
    for i in range(0, len(raw), chunk):
        screen.feed(raw[i : i + chunk], now=100)
    return screen.latest


def test_real_gui_captures_survive_split_escape_sequences():
    for board, role in [("spec", "SLAVE"), ("acorn", "MASTER")]:
        expected = captured_frame(board, 100000)
        assert expected["role"] == role
        assert expected["detection"] == "EXT_ON"
        for size in (1, 7, 127):
            assert captured_frame(board, size) == expected
    assert captured_frame("spec", 7)["servo"] == "TRACK_PHASE"


def test_transient_down_or_unknown_port_rows_are_not_discarded():
    raw = (ROOT / "test/fixtures/spec-wr-tracking.uart").read_bytes()
    screen = status.WRScreen()
    screen.feed(raw, now=100)
    frames = screen.feed(b"\x1b[12;26f\x1b[K\x1b[33;1f\x1b[J", now=101)
    assert len(frames) == 1
    assert frames[0]["role"] is None and frames[0]["peer"] is None
    frames = screen.feed(b"\x1b[12;46fFAULTY /IDLE /EXT_ON\x1b[33;1f\x1b[J", now=102)
    assert frames[0]["role"] == "FAULTY"


def test_qualification_rejects_ptp_fallback_staleness_and_unlocked_servo():
    master = captured_frame("acorn", 7)
    slave = captured_frame("spec", 7)
    md = dict(ptp_state=6, link=True, locked=True, tai_ns=10_000_000_000)
    sd = dict(
        ptp_state=9, link=True, locked=True, servo_valid=True, servo_state=4, tai_ns=10_500_000_000
    )
    assert status.qualification_errors(master, slave, md, sd, now=101) == []
    for field, value in [
        ("detection", "EXT_OFF"),
        ("extension", "WR_S_LOCK"),
        ("servo", "<NULL>"),
        ("role", "MASTER"),
        ("pll_locked", False),
        ("frequency_locked", False),
        ("received", 90),
        ("peer", "00:00:00:00:00:00"),
    ]:
        changed = deepcopy(slave)
        changed[field] = value
        assert status.qualification_errors(master, changed, md, sd, now=101), field
    # The firmware's misleading WR_MODE bit is still set in this PTP fallback.
    plain_ptp = deepcopy(slave)
    plain_ptp.update(detection="EXT_OFF", servo="<NULL>")
    assert status.qualification_errors(master, plain_ptp, md, sd, now=101)
    for field, value in [
        ("ptp_state", 8),
        ("link", False),
        ("locked", False),
        ("servo_valid", False),
        ("servo_state", 5),
        ("tai_ns", 0),
    ]:
        changed = deepcopy(sd)
        changed[field] = value
        assert status.qualification_errors(master, slave, md, changed, now=101), field
