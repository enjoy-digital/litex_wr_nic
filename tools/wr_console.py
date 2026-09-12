#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#

"""Paced UART commands and complete WR console capture."""

import json
from pathlib import Path
import re
import threading
import time

import serial


ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
# PPSI diagnostics can be printed between any two shell echo characters.
# Strip only their timestamped records when matching an echo; retain the full
# response and raw UART capture for diagnosis.
PPSI_DIAGNOSTIC = re.compile(r"diag-[\w-]+: \d+\.\d+: [^\n]*\n\n?")


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
            echo_text = PPSI_DIAGNOSTIC.sub("", response)
            after_echo = 0 if command is None else echo_text.find(command + "\n")
            if after_echo >= 0 and "wrc#" in echo_text[after_echo:]:
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
