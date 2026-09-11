#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#

"""WRPC v5 GUI frames and coherent host diagnostic snapshots."""

import re
import time

CSI = re.compile(r"\x1b\[([0-?]*)([ -/]*)([@-~])")


class WRScreen:
    """Small terminal model for WRPC's cursor-addressed monitor, including split CSI."""

    def __init__(self):
        self.rows = [[" "] * 100 for _ in range(50)]
        self.row = self.col = 0
        self.pending = ""
        self.latest = None

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        text = self.pending + data.decode("ascii", errors="replace")
        self.pending = ""
        i = 0
        frames = []
        while i < len(text):
            ch = text[i]
            if ch == "\x1b":
                match = CSI.match(text, i)
                if match is None:
                    self.pending = text[i:]
                    break
                values, _, command = match.groups()
                args = (
                    [int(x or 0) for x in values.split(";")] if not values.startswith("?") else []
                )
                n = args[0] if args else 0
                if command in ("H", "f"):
                    self.row = max(0, min(49, (n or 1) - 1))
                    self.col = max(0, min(99, ((args[1] if len(args) > 1 else 1) or 1) - 1))
                elif command == "J":
                    if n == 2:
                        self.rows = [[" "] * 100 for _ in range(50)]
                        self.latest = None
                    elif n == 0:
                        self.rows[self.row][self.col :] = [" "] * (100 - self.col)
                        for row in range(self.row + 1, 50):
                            self.rows[row] = [" "] * 100
                        frame = self.frame(now)
                        if frame is not None:
                            self.latest = frame
                            frames.append(frame)
                elif command == "K":
                    if n == 0:
                        self.rows[self.row][self.col :] = [" "] * (100 - self.col)
                    elif n == 2:
                        self.rows[self.row] = [" "] * 100
                elif command == "@":
                    count = n or 1
                    self.rows[self.row][self.col :] = (
                        [" "] * count + self.rows[self.row][self.col :]
                    )[: 100 - self.col]
                i = match.end()
                continue
            if ch == "\r":
                self.col = 0
            elif ch == "\n":
                self.row = min(49, self.row + 1)
            elif ch == "\b":
                self.col = max(0, self.col - 1)
            elif ord(ch) >= 32:
                self.rows[self.row][self.col] = ch
                self.col = min(99, self.col + 1)
            i += 1
        return frames

    def frame(self, now):
        rows = ["".join(row) for row in self.rows]
        tai = re.search(r"\d{4}-\d{2}-\d{2}-\d{2}:\d{2}:\d{2}", rows[2])
        port = re.search(r"\b(\w+)\s*/\s*(\w+)\s*/\s*(\w+)", rows[11])
        mac = re.search(r"(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}", rows[6])
        peer = re.search(r"(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}", rows[11])
        # The static redraw also ends in ESC[J before values are populated.
        # Once time and our MAC are present, retain even malformed/down port
        # rows so a brief loss cannot disappear between good frames.
        if not (tai and mac):
            return None
        servo = re.search(r"White-Rabbit:\s*(\S+)", rows[15])
        count = re.search(r"(\d+) times", rows[29])
        return dict(
            received=now,
            tai=tai.group(),
            mac=mac.group().lower(),
            peer=peer.group().lower() if peer else None,
            role=port[1] if port else None,
            extension=port[2] if port else None,
            detection=port[3] if port else None,
            pll_locked=rows[2][69:76].strip() == "Locked",
            frequency_locked=rows[11][7:10] == "Lck",
            servo=servo[1] if servo else None,
            updates=int(count[1]) if count else None,
            screen="\n".join(row.rstrip() for row in rows).rstrip(),
        )


def diagnostic_snapshot(bus, timeout=3):
    base = bus.mems.wr_wb_slave.base + 0x900
    if bus.read(base) != 2:
        raise RuntimeError("WR diagnostic version is not 2; verify the CPU device map")
    try:
        # Clear VALID while requesting a new snapshot; firmware sets it only
        # after completing a fresh update, then holds all fields for the host.
        bus.write(base + 4, 0x100)
        deadline = time.monotonic() + timeout
        while bus.read(base + 4) & 0x101 != 0x101:
            if time.monotonic() > deadline:
                raise TimeoutError("WR diagnostics did not produce a fresh snapshot")
            time.sleep(0.02)
        words = bus.read(base, length=25)
        if words[1] & 0x101 != 0x101:
            raise RuntimeError("WR diagnostic snapshot lost validity")
    finally:
        bus.write(base + 4, 0)
    signed = lambda value: value if value < 0x80000000 else value - 0x100000000
    if words[10] >= 1_000_000_000:
        raise RuntimeError("WR diagnostic time has an invalid nanosecond field")
    return dict(
        version=words[0],
        raw=words,
        # Firmware calls this WR_MODE but populates it with servo validity.
        # Actual WR extension activation must be checked independently via GUI.
        servo_valid=bool(words[2] & 1),
        servo_state=(words[2] >> 8) & 15,
        link=bool(words[3] & 1),
        locked=bool(words[3] & 2),
        ptp_state=words[4] & 255,
        tai_ns=((words[8] << 32) | words[9]) * 1000000000 + words[10],
        offset_ps=signed(words[16]),
        updates=words[18],
        tx=words[6],
        rx=words[7],
        rx_errors=words[24],
    )


def qualification_errors(master, slave, master_diag, slave_diag, now):
    errors = []
    for name, frame, diag, role, state in (
        ("master", master, master_diag, "MASTER", 6),
        ("slave", slave, slave_diag, "SLAVE", 9),
    ):
        if frame is None:
            errors.append(name + ": no complete GUI frame")
            continue
        if now - frame["received"] > 5:
            errors.append(name + ": stale UART monitor")
        if frame["role"] != role or diag["ptp_state"] != state:
            errors.append(name + ": incorrect PTP role")
        if frame["detection"] != "EXT_ON" or frame["extension"] != "IDLE":
            errors.append(name + ": WR extension is not active and idle")
        if not (
            frame["pll_locked"] and frame["frequency_locked"] and diag["link"] and diag["locked"]
        ):
            errors.append(name + ": link/frequency lock missing")
    if (
        master
        and slave
        and (
            master["mac"] != slave["peer"]
            or slave["mac"] != master["peer"]
            or master["mac"] == slave["mac"]
        )
    ):
        errors.append("peer MAC identities do not agree")
    if slave and (
        slave["servo"] != "TRACK_PHASE"
        or not slave_diag["servo_valid"]
        or slave_diag["servo_state"] != 4
    ):
        errors.append("slave: servo is not tracking WR phase")
    # Separate firmware refresh periods prevent a precise timestamp comparison,
    # but both boards must agree on the time scale.
    if abs(master_diag["tai_ns"] - slave_diag["tai_ns"]) > 3_000_000_000:
        errors.append("master/slave diagnostic time scales disagree")
    return errors
