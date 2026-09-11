#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#

"""Capture physical WR consoles and qualify a pair of White Rabbit boards."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import threading
import time

import serial


ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


class Console:
    def __init__(self, port, output, baudrate=115200):
        output = Path(output)
        output.parent.mkdir(parents=True, exist_ok=True)
        self.port = serial.Serial(port, baudrate, timeout=0.1, write_timeout=2, exclusive=True)
        self.raw = output.with_suffix(".uart.raw").open("wb", buffering=0)
        self.events = output.with_suffix(".events.jsonl").open("w", buffering=1)
        self.buffer = bytearray()
        self.last_received = None
        self.error = None
        self.running = True
        self.reader = threading.Thread(target=self.read, daemon=True)
        self.reader.start()

    def read(self):
        try:
            while self.running:
                data = self.port.read(4096)
                if data:
                    self.raw.write(data)
                    self.buffer.extend(data)
                    self.last_received = time.monotonic()
        except Exception as error:
            if self.running:
                self.error = error

    def text(self, start=0):
        if self.error is not None:
            raise RuntimeError("UART disconnected") from self.error
        return ANSI.sub("", bytes(self.buffer[start:]).decode(errors="replace")).replace("\r", "")

    def record(self, kind, text):
        self.events.write(json.dumps({"time": time.time(), "kind": kind, "text": text}) + "\n")

    def send(self, text):
        self.record("tx", text)
        # WRPC polls a small RX FIFO; pace commands so USB bursts do not lose bytes.
        for value in text.encode():
            self.port.write(bytes([value]))
            time.sleep(0.02)

    def prompt(self, start, timeout=10, command=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            response = self.text(start)
            # Asynchronous WR logs can redraw an old prompt while a command is
            # being typed. Require the submitted command's echo before accepting
            # its completion prompt; otherwise a later command can race it.
            after_echo = 0 if command is None else response.find(command + "\n")
            if after_echo >= 0 and "wrc#" in response[after_echo:]:
                self.record("rx", response)
                return response
            time.sleep(0.05)
        raise TimeoutError("WR prompt timeout: " + repr(self.text(start)[-1000:]))

    def connect(self):
        start = len(self.buffer)
        self.send("\r")
        try:
            return self.prompt(start, timeout=3)
        except TimeoutError:
            start = len(self.buffer)
            self.send("\x1bq\r")
            return self.prompt(start)

    def command(self, command, timeout=10):
        start = len(self.buffer)
        self.send(command + "\r")
        response = self.prompt(start, timeout, command=command)
        if re.search(r'Unrecognized command|Unknown subcommand|Command "[^"\n]+": error', response):
            raise RuntimeError("WR command failed: " + response)
        return response

    def monitor(self, seconds):
        start = len(self.buffer)
        self.send("gui\r")
        time.sleep(seconds)
        screen = self.text(start)
        self.record("gui", screen)
        start = len(self.buffer)
        self.send("q")
        self.prompt(start)
        return screen

    def close(self):
        self.running = False
        self.reader.join(timeout=2)
        self.port.close()
        self.raw.close()
        self.events.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="action", required=True)
    console_parser = subparsers.add_parser(
        "console", help="Capture commands on one physical WR UART."
    )
    console_parser.add_argument("--port", required=True)
    console_parser.add_argument("--output", type=Path, required=True)
    console_parser.add_argument("--command", action="append", default=[])
    console_parser.add_argument("--monitor-seconds", type=float, default=0)
    console_parser.add_argument("--load-bitstream", type=Path)
    console_parser.add_argument(
        "--usb-busdev", help="Explicit openFPGALoader USB bus:device selector."
    )
    jtag_parser = subparsers.add_parser(
        "jtag", help="Serve one board with independently selected JTAG/TCP ports."
    )
    jtag_parser.add_argument("--config", required=True)
    jtag_parser.add_argument("--port", type=int, required=True)
    jtag_parser.add_argument("--stream-port", type=int, required=True)
    status_parser = subparsers.add_parser(
        "status", help="Record identity, WR CSRs and measured clocks."
    )
    status_parser.add_argument("--port", type=int, required=True)
    status_parser.add_argument("--csr-csv", required=True)
    status_parser.add_argument("--output", type=Path, required=True)
    status_parser.add_argument(
        "--read", action="append", type=lambda value: int(value, 0), default=[]
    )
    args = parser.parse_args()
    if args.action == "status":
        from litex import RemoteClient

        bus = RemoteClient(
            host="127.0.0.1", port=args.port, csr_csv=args.csr_csv, timeout=5, raise_on_timeout=True
        )
        result = {"port": args.port, "registers": {}, "reads": {}}
        bus.open()
        try:
            words = bus.read(bus.bases.identifier_mem, length=128)
            result["identifier"] = (
                bytes(word & 255 for word in words).split(b"\0")[0].decode(errors="replace")
            )
            for name, reg in vars(bus.regs).items():
                if name.startswith(
                    (
                        "wr_",
                        "refclk_dac_",
                        "dmtd_dac_",
                        "refclk_mmcm_",
                        "dmtd_mmcm_",
                        "uart_control",
                    )
                ):
                    result["registers"][name] = reg.read()
            for address in args.read:
                result["reads"][hex(address)] = bus.read(address)
            samples = []
            for _ in range(2):
                for i in range(3):
                    getattr(bus.regs, f"clk_measurement_clk{i}_latch").write(1)
                time.sleep(0.1)
                samples.append(
                    {
                        "time": time.monotonic(),
                        "values": [
                            getattr(bus.regs, f"clk_measurement_clk{i}_value").read()
                            for i in range(3)
                        ],
                    }
                )
                if len(samples) == 1:
                    time.sleep(1)
            elapsed = samples[1]["time"] - samples[0]["time"]
            result["clocks_hz"] = [
                ((b - a) % (1 << 64)) / elapsed
                for a, b in zip(samples[0]["values"], samples[1]["values"])
            ]
            result["clock_samples"] = samples
        finally:
            bus.close()
            args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        return
    if args.action == "jtag":
        from litex.tools.litex_term import JTAGUART
        from litex.tools.litex_server import RemoteServer
        from litex.tools.remote.comm_uart import CommUART

        uart = JTAGUART(config=args.config, port=args.stream_port)
        uart.open()
        comm = CommUART(os.ttyname(uart.name))
        server = RemoteServer(comm, "127.0.0.1", args.port)
        try:
            server.open()
            server.start(4)
            while True:
                time.sleep(1)
        finally:
            server.close()
            uart.close()
        return
    if args.load_bitstream and not args.usb_busdev:
        parser.error("--load-bitstream requires --usb-busdev")
    console = Console(args.port, args.output)
    result = {"port": args.port, "commands": [], "passed": False}
    try:
        if args.load_bitstream:
            result["bitstream_sha256"] = hashlib.sha256(
                args.load_bitstream.read_bytes()
            ).hexdigest()
            with args.output.with_suffix(".program.log").open("w") as log:
                subprocess.run(
                    [
                        "openFPGALoader",
                        "--cable",
                        "ft4232",
                        "--busdev-num",
                        args.usb_busdev,
                        "--freq",
                        "5000000",
                        "--bitstream",
                        str(args.load_bitstream),
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    check=True,
                    timeout=90,
                )
            result["boot"] = console.prompt(0, timeout=60)
            print(result["boot"], flush=True)
        result["connect"] = console.connect()
        for command in args.command:
            response = console.command(command)
            result["commands"].append({"command": command, "response": response})
            print(response, flush=True)
        if args.monitor_seconds:
            result["monitor"] = console.monitor(args.monitor_seconds)
            print(result["monitor"], flush=True)
        result["passed"] = True
    except Exception as error:
        result["error"] = str(error)
        raise
    finally:
        console.close()
        args.output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
