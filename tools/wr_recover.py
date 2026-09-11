#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#

"""Exercise volatile reload and PCS link recovery on two USB-connected WR boards."""

import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

from litex import RemoteClient
from wr_link import Console
from wr_status import diagnostic_snapshot
from wr_urv_debug import URVDebug


class Board:
    def __init__(self, name, config):
        self.name, self.config = name, config
        self.server = self.server_log = None

    def client(self):
        bus = RemoteClient(
            host="127.0.0.1",
            port=self.config["jtag_port"],
            csr_csv=self.config["csr"],
            timeout=5,
            raise_on_timeout=True,
        )
        bus.open()
        if bus.read(bus.mems.wr_wb_slave.base) != 0x57525043:
            bus.close()
            raise RuntimeError(self.name + ": wrong WR host map")
        return bus

    def start_server(self, output):
        with socket.socket() as probe:
            if probe.connect_ex(("127.0.0.1", self.config["jtag_port"])) == 0:
                raise RuntimeError(self.name + ": stop the existing JTAG server first")
        self.server_log = output.open("w")
        self.server = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "litex.tools.litex_server",
                "--jtag",
                "--jtag-config",
                self.config["jtag_config"],
                "--jtag-port",
                str(self.config["stream_port"]),
                "--bind-ip",
                "127.0.0.1",
                "--bind-port",
                str(self.config["jtag_port"]),
            ],
            stdout=self.server_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if self.server.poll() is not None:
                raise RuntimeError(self.name + ": JTAG server exited; inspect " + str(output))
            with socket.socket() as probe:
                if probe.connect_ex(("127.0.0.1", self.config["jtag_port"])) == 0:
                    bus = self.client()
                    bus.close()
                    return
            time.sleep(0.1)
        raise TimeoutError(self.name + ": JTAG server startup")

    def stop_server(self):
        if self.server is None:
            return
        # This process group was created by start_server. OpenOCD's streaming
        # loop can ignore SIGTERM, so also reap that owned child after a grace period.
        try:
            os.killpg(self.server.pid, signal.SIGTERM)
            try:
                self.server.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass
            try:
                os.killpg(self.server.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.server.wait(timeout=5)
        except ProcessLookupError:
            pass
        self.server_log.close()
        self.server = self.server_log = None

    def reload(self, output):
        self.stop_server()
        cfg = self.config
        usb = Path("/sys/bus/usb/devices") / cfg["usb_location"]
        if [(usb / key).read_text().strip() for key in ("idVendor", "idProduct")] != [
            "0403",
            "6011",
        ]:
            raise RuntimeError(self.name + ": USB location is not the expected FT4232H")
        uart = Path("/sys/class/tty") / Path(cfg["uart"]).resolve().name / "device"
        if cfg["usb_location"] not in [parent.name for parent in uart.resolve().parents]:
            raise RuntimeError(self.name + ": UART and JTAG USB locations disagree")
        busdev = ":".join(str(int((usb / key).read_text())) for key in ("busnum", "devnum"))
        bitstream = Path(cfg["bitstream"])
        result = dict(
            board=self.name,
            usb_location=cfg["usb_location"],
            usb_busdev=busdev,
            bitstream=str(bitstream),
            sha256=hashlib.sha256(bitstream.read_bytes()).hexdigest(),
            started_unix=time.time(),
            passed=False,
        )
        console = Console(cfg["uart"], output)
        try:
            with output.with_suffix(".program.log").open("w") as log:
                subprocess.run(
                    [
                        "openFPGALoader",
                        "--cable",
                        "ft4232",
                        "--busdev-num",
                        busdev,
                        "--freq",
                        "5000000",
                        "--bitstream",
                        str(bitstream),
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    check=True,
                    timeout=90,
                )
            result["programmed_unix"] = time.time()
            result["programmed_monotonic"] = time.monotonic()
            result["boot"] = console.prompt(0, timeout=60)
            console.connect()
            result["version"] = console.command("ver")
            result["passed"] = True
        except BaseException as error:
            result["error"] = str(error)
            raise
        finally:
            console.close()
            result["finished_unix"] = time.time()
            output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
        self.start_server(output.with_suffix(".jtag.log"))
        return result


def interrupt_link(board, peer, output, seconds):
    """Assert the PCS MDIO power-down bit, prove link-down, then restore MCR.

    This resets the SERDES/PCS; it does not emulate physical removal of the
    module or exercise connector contacts. PTP remains under firmware control.
    """
    buses, consoles = {}, {}
    result = dict(passed=False, board=board.name, peer=peer.name, samples=[])
    original = None
    cleanup = ExitStack()
    try:
        for item in (board, peer):
            buses[item.name] = item.client()
            cleanup.callback(buses[item.name].close)
            c = consoles[item.name] = Console(item.config["uart"], output / item.name)
            cleanup.callback(c.close)
            c.connect()
            c.command("verbose 1")
        bus = buses[board.name]
        # The host map exposes only endpoint MAC registers, not MDIO. Access
        # the CPU peripheral map through uRV's existing debug instruction port.
        # Each context briefly pauses the slave CPU, preserves a0/a1, and resumes
        # it before observing link-down or recovery. No firmware image is changed.
        ep = 0x100100

        def mdio_ready(cpu):
            deadline = time.monotonic() + 2
            while True:
                value = cpu.read(ep + 0x30)
                if value & 0x80000000:
                    return value & 0xFFFF
                if time.monotonic() > deadline:
                    raise TimeoutError("PCS MDIO transaction timed out")

        def write_mcr(cpu, value):
            cpu.write(ep + 0x30, 0)
            cpu.write(ep + 0x2C, 0x80000000 | value)
            mdio_ready(cpu)

        try:
            with URVDebug(bus) as cpu:
                cpu.write(ep + 0x30, 0)
                cpu.write(ep + 0x2C, 0)
                value = mdio_ready(cpu)
                if value != 0x1140:
                    raise RuntimeError(f"Unexpected PCS MCR: 0x{value:04x}")
                original = value
                result["mcr_original"] = original
                write_mcr(cpu, original | 0x800)
            result["disable_cpu_pause_seconds"] = cpu.paused_seconds
            result["disabled_unix"] = time.time()
            start = time.monotonic()
            for name, c in consoles.items():
                result[name + "_during_down_version"] = c.command("ver")
            while not result["samples"] or time.monotonic() - start < seconds:
                sample = {name: diagnostic_snapshot(client) for name, client in buses.items()}
                result["samples"].append(sample)
            if not any(
                all(not state["link"] for state in sample.values()) for sample in result["samples"]
            ):
                raise RuntimeError(
                    "PCS power-down did not produce observed link-down on both boards"
                )
        finally:
            # Restore even if a capture or assertion failed. A failed restore
            # propagates and must never produce a passing interruption record.
            if original is not None:
                with URVDebug(bus) as cpu:
                    write_mcr(cpu, original)
                result["restore_cpu_pause_seconds"] = cpu.paused_seconds
                result["enabled_unix"] = time.time()
                result["enabled_monotonic"] = time.monotonic()
        for name, c in consoles.items():
            result[name + "_ptp"] = c.command("ptp")
        result["passed"] = True
    except BaseException as error:
        result["error"] = str(error)
        raise
    finally:
        try:
            cleanup.close()
        finally:
            (output / "interruption.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, required=True, help="JSON with exactly two named board records."
    )
    parser.add_argument("--master", required=True)
    parser.add_argument("--master-trim", type=int)
    parser.add_argument("--operation", choices=["link", "reload", "both"], default="both")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--link-down-seconds", type=float, default=5)
    parser.add_argument("--recovered-seconds", type=float, default=10)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    if len(config) != 2 or args.master not in config:
        parser.error("The configuration must contain the master and exactly one peer")
    if args.cycles < 1 or args.link_down_seconds < 3 or args.recovered_seconds <= 0:
        parser.error("Use positive cycles/observation time and at least 3 seconds link-down")
    args.output.mkdir(parents=True, exist_ok=False)
    boards = {name: Board(name, cfg) for name, cfg in config.items()}
    master = boards[args.master]
    slave = next(board for name, board in boards.items() if name != args.master)
    summary = dict(passed=False, master=master.name, slave=slave.name, results=[])

    def qualify(output, configure=False):
        cmd = [
            sys.executable,
            str(Path(__file__).with_name("wr_qualify.py")),
            "--output",
            str(output),
            "--duration",
            str(args.recovered_seconds),
        ]
        for role, board in [("master", master), ("slave", slave)]:
            cmd += [
                "--" + role + "-uart",
                board.config["uart"],
                "--" + role + "-csr",
                board.config["csr"],
                "--" + role + "-jtag",
                str(board.config["jtag_port"]),
            ]
        if configure:
            cmd += ["--configure"]
            if args.master_trim is not None:
                cmd += ["--master-trim", str(args.master_trim)]
        with output.with_suffix(".log").open("w") as log:
            subprocess.run(
                cmd,
                stdout=log,
                stderr=subprocess.STDOUT,
                check=True,
                timeout=420 + args.recovered_seconds,
            )
        return json.loads((output / "summary.json").read_text())

    try:
        for board in boards.values():
            board.start_server(args.output / (board.name + "-initial-jtag.log"))
        summary["initial"] = qualify(args.output / "initial")
        operations = ["link", "reload"] if args.operation == "both" else [args.operation]
        for operation in operations:
            for cycle in range(1, args.cycles + 1):
                output = args.output / f"{operation}-{cycle}"
                output.mkdir()
                result = dict(operation=operation, cycle=cycle)
                if operation == "link":
                    result["interruption"] = interrupt_link(
                        slave, master, output, args.link_down_seconds
                    )
                    ready = result["interruption"]["enabled_monotonic"]
                else:
                    result["reloads"] = [
                        board.reload(output / board.name) for board in boards.values()
                    ]
                    ready = result["reloads"][-1]["programmed_monotonic"]
                result["qualification"] = qualify(
                    output / "reacquired", configure=operation == "reload"
                )
                # Include command setup/observer startup, not only its GUI loop.
                result["recovery_upper_bound_seconds"] = (
                    time.monotonic() - ready - result["qualification"]["soak_seconds"]
                )
                summary["results"].append(result)
                (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
                if result["recovery_upper_bound_seconds"] > 300:
                    raise RuntimeError("Recovery exceeded five minutes")
                print(
                    f"{operation} {cycle}/{args.cycles} passed; recovery <= {result['recovery_upper_bound_seconds']:.1f}s",
                    flush=True,
                )
        summary["passed"] = True
    except BaseException as error:
        summary["error"] = str(error)
        raise
    finally:
        for board in boards.values():
            board.stop_server()
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")


if __name__ == "__main__":
    main()
