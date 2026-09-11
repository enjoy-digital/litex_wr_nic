#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#

"""Qualify two configured WR boards with physical UART and JTAGBone evidence."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import time

from litex import RemoteClient
from wr_link import Console
from wr_status import WRScreen, diagnostic_snapshot, qualification_errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for role in ("master", "slave"):
        parser.add_argument("--" + role + "-uart", required=True)
        parser.add_argument("--" + role + "-jtag", required=True, type=int)
        parser.add_argument("--" + role + "-csr", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=1800)
    parser.add_argument("--acquire-timeout", type=float, default=300)
    parser.add_argument(
        "--configure",
        action="store_true",
        help="Set roles before acquisition; otherwise observe the running pair.",
    )
    parser.add_argument(
        "--master-trim",
        type=int,
        help="Apply a volatile master main-DAC code after role configuration.",
    )
    args = parser.parse_args()
    if args.duration <= 0 or args.acquire_timeout <= 0:
        parser.error("Durations must be positive")
    if args.master_trim is not None and not (args.configure and 0 <= args.master_trim <= 65535):
        parser.error("--master-trim requires --configure and a 16-bit code")
    args.output.mkdir(parents=True, exist_ok=False)
    summary = dict(
        passed=False,
        start_unix=time.time(),
        duration_requested=args.duration,
        arguments={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        boards={},
        tool_sha256={
            name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
            for name in ("wr_qualify.py", "wr_link.py", "wr_status.py")
        },
    )
    consoles, buses, screens, offsets = {}, {}, {}, {}
    samples = (args.output / "samples.jsonl").open("w", buffering=1)
    frames_log = (args.output / "gui-frames.jsonl").open("w", buffering=1)
    pool = ThreadPoolExecutor(max_workers=2)
    try:
        for role in ("master", "slave"):
            csv = getattr(args, role + "_csr")
            summary["boards"][role] = dict(
                csr_sha256=hashlib.sha256(csv.read_bytes()).hexdigest(), commands=[]
            )
            c = consoles[role] = Console(getattr(args, role + "_uart"), args.output / role)
            c.connect()
            b = buses[role] = RemoteClient(
                host="127.0.0.1",
                port=getattr(args, role + "_jtag"),
                csr_csv=str(csv),
                timeout=5,
                raise_on_timeout=True,
            )
            b.open()
            identifier = (
                bytes(w & 255 for w in b.read(b.bases.identifier_mem, length=128))
                .split(b"\0")[0]
                .decode()
            )
            summary["boards"][role]["identifier"] = identifier
            if b.read(b.mems.wr_wb_slave.base) != 0x57525043:
                raise RuntimeError(role + ": WR host map identity mismatch")

        def command(role, text):
            response = consoles[role].command(text, timeout=60)
            summary["boards"][role]["commands"].append(dict(command=text, response=response))
            if "PRINTF OVF" in response:
                raise RuntimeError(role + ": firmware printf buffer overflow")
            return response

        for role in consoles:
            command(role, "verbose 0")
            command(role, "ver")
        if args.configure:
            for role in consoles:
                command(role, "ptp stop")
            for role in consoles:
                command(role, "mode " + role)
            if args.master_trim is not None:
                command("master", f"pll sdac 0 {args.master_trim}")
            for role in consoles:
                command(role, "ptp start")
        for role, c in consoles.items():
            command(role, "ptp")
            command(role, "mac get")
            screens[role] = WRScreen()
            offsets[role] = len(c.buffer)
            c.send("gui\r")
        start = time.monotonic()
        tracking_start = None
        previous = None
        progress_times = {role: start for role in consoles}
        update_progress = start
        last_report = start - 30
        while True:
            new_frames = {}
            for role, c in consoles.items():
                if c.error:
                    raise RuntimeError(role + ": UART reader failed") from c.error
                end = len(c.buffer)
                # Retain enough overlap to catch a warning split across reads.
                if b"PRINTF OVF" in c.buffer[max(0, offsets[role] - 9) : end]:
                    raise RuntimeError(role + ": firmware printf buffer overflow")
                new_frames[role] = screens[role].feed(
                    bytes(c.buffer[offsets[role] : end]), now=c.last_received or start
                )
                offsets[role] = end
                for frame in new_frames[role]:
                    frames_log.write(json.dumps(dict(board_role=role, **frame)) + "\n")
            pending = {role: pool.submit(diagnostic_snapshot, b) for role, b in buses.items()}
            diags = {role: future.result() for role, future in pending.items()}
            now = time.monotonic()
            master, slave = screens["master"].latest, screens["slave"].latest
            errors = qualification_errors(master, slave, diags["master"], diags["slave"], now)
            # Every complete UART frame during the soak is checked, including
            # transient losses between host diagnostic samples.
            if tracking_start is not None:
                for frame in new_frames["master"]:
                    errors += qualification_errors(
                        frame, slave, diags["master"], diags["slave"], now
                    )
                for frame in new_frames["slave"]:
                    errors += qualification_errors(
                        master, frame, diags["master"], diags["slave"], now
                    )
                for role in diags:
                    if diags[role]["tai_ns"] < previous[role]["tai_ns"]:
                        errors.append(role + ": TAI moved backwards")
                    if diags[role]["tai_ns"] != previous[role]["tai_ns"]:
                        progress_times[role] = now
                    if now - progress_times[role] > 5:
                        errors.append(role + ": TAI stopped advancing")
                    if diags[role]["rx_errors"] != previous[role]["rx_errors"]:
                        errors.append(role + ": RX error count changed")
                if diags["slave"]["updates"] != previous["slave"]["updates"]:
                    update_progress = now
                if (
                    diags["slave"]["updates"] - previous["slave"]["updates"]
                ) & 0xFFFFFFFF > 0x80000000:
                    errors.append("slave: servo update counter moved backwards")
                if now - update_progress > 10:
                    errors.append("slave: servo update counter stopped advancing")
            sample = dict(
                elapsed=now - start,
                soak_elapsed=None if tracking_start is None else now - tracking_start,
                errors=sorted(set(errors)),
                diagnostics=diags,
                gui={
                    role: {k: v for k, v in screens[role].latest.items() if k != "screen"}
                    if screens[role].latest
                    else None
                    for role in screens
                },
            )
            samples.write(json.dumps(sample) + "\n")
            if tracking_start is None:
                if not errors:
                    tracking_start = now
                    summary["acquired_after"] = now - start
                    progress_times = {role: now for role in diags}
                    update_progress = now
                    summary["initial_diagnostics"] = diags
                    print(
                        f"WR tracking acquired after {now - start:.1f}s; starting uninterrupted {args.duration:.0f}s soak",
                        flush=True,
                    )
                elif now - start > args.acquire_timeout:
                    raise RuntimeError("WR acquisition timeout: " + ", ".join(sorted(set(errors))))
            elif errors:
                raise RuntimeError("WR soak interrupted: " + ", ".join(sorted(set(errors))))
            elif now - tracking_start >= args.duration:
                summary.update(
                    passed=True, soak_seconds=now - tracking_start, final_diagnostics=diags
                )
                break
            previous = diags
            if now - last_report >= 30:
                print(
                    json.dumps(
                        dict(
                            elapsed=round(now - start, 1),
                            soak=sample["soak_elapsed"],
                            errors=sample["errors"],
                            slave_offset_ps=diags["slave"]["offset_ps"],
                            slave_updates=diags["slave"]["updates"],
                        )
                    ),
                    flush=True,
                )
                last_report = now
            time.sleep(0.1)
    except BaseException as error:
        summary["error"] = str(error)
        raise
    finally:
        pool.shutdown(wait=True)
        for c in consoles.values():
            try:
                start = len(c.buffer)
                c.send("q")
                c.prompt(start)
            except Exception:
                pass
            c.close()
        for b in buses.values():
            b.close()
        samples.close()
        frames_log.close()
        summary["end_unix"] = time.time()
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
