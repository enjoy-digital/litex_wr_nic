#!/usr/bin/env python3
#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Measure an already loaded spec_a7_mmcm image through a LiteX server."""

import argparse
import json
import time
from pathlib import Path

from litex import RemoteClient

# Measurement --------------------------------------------------------------------------------------

class Channel:
    def __init__(self, bus, name):
        self.bus  = bus
        self.name = name
        self.gate = self.read("gate_cycles")
        self.freq = self.read("frequency")
        self.vco  = self.read("vco")

    def read(self, name):
        return getattr(self.bus.regs, f"{self.name}_{name}").read()

    def write(self, name, value):
        getattr(self.bus.regs, f"{self.name}_{name}").write(value)

    def reset(self):
        self.write("enable", 0)
        self.write("reset", 1)
        self.write("pause", 0)
        self.write("drop_done", 0)
        self.write("code", 32768)
        time.sleep(0.01)
        self.write("reset", 0)
        deadline = time.monotonic() + 3
        while not self.read("locked"):
            assert time.monotonic() < deadline, f"{self.name}: MMCM did not lock"
            time.sleep(0.01)
        assert self.read("backend_status") == 0
        assert self.read("errors") == 0

    def measure(self):
        previous = self.read("sample_id")
        self.write("measure", 1)
        deadline = time.monotonic() + 3
        while self.read("sample_id") == previous:
            assert time.monotonic() < deadline, f"{self.name}: measurement timed out"
            time.sleep(0.02)
        result = {name: self.read(f"sample_{name}") for name in
            ("id", "edges", "phase", "issued", "completed", "steps", "errors")}
        if result["phase"] >= 2**31:
            result["phase"] -= 2**32
        result.update(channel=self.name, frequency=self.freq, vco=self.vco, gate=self.gate)
        assert result["errors"] == 0, result
        assert self.read("locked"), result
        expected = self.gate*self.freq/200e6 - result["phase"]*self.freq/(56*self.vco)
        result["expected_edges"] = expected
        result["edge_error"]     = result["edges"] - expected
        # Gray synchronization and the two endpoints quantize this count.
        assert abs(result["edge_error"]) <= 4, result
        return result

    def check_rate(self, code, period):
        self.write("period", period)
        self.write("code", code)
        self.write("enable", 1)
        time.sleep(0.01)
        result = self.measure()
        result.update(code=code, period=period)
        expected = self.gate*abs(code - 32768)/2**19
        result["expected_steps"] = expected
        assert abs(result["steps"] - expected) <= 1, result
        assert abs(result["issued"] - result["completed"]) <= 1, result
        assert abs(result["steps"] - result["completed"]) <= 1, result
        assert result["phase"] == (1 if code < 32768 else -1)*result["completed"], result
        assert not self.read("backend_status") & 2, result
        return result

# Qualification ------------------------------------------------------------------------------------

def read_identifier(bus):
    contents = []
    for index in range(256):
        byte = bus.read(bus.bases.identifier_mem + 4*index) & 0xff
        if byte == 0:
            return bytes(contents).decode("ascii")
        contents.append(byte)
    raise ValueError("Unterminated FPGA identifier")


def qualify(bus, record):
    identifier = read_identifier(bus)
    assert identifier == "WR MMCM hardware qualification", identifier
    channels = [Channel(bus, name) for name in ("ref100", "dmtd100", "ref200", "dmtd200")]
    for channel in channels:
        channel.write("enable", 0)
        channel.write("reset", 1)
    for channel in channels:
        channel.reset()
        # Every-cycle refresh includes the one-LSB regression that previously
        # discarded fractional phase. A slower cadence checks the same gain.
        for period in (1, 4096):
            for code in (32768, 32767, 32769, 0, 65535, 16384, 49152, 32768):
                record(channel.check_rate(code, period))

        # Missing completion must stop requests and keep a sticky fault.
        channel.reset()
        channel.write("drop_done", 1)
        channel.write("code", 0)
        channel.write("enable", 1)
        time.sleep(0.02)
        assert channel.read("backend_status") == 3
        channel.write("code", 65535)
        result = channel.measure()
        assert result["issued"] == result["completed"] == result["steps"] == 0, result
        assert channel.read("backend_status") == 3
        result["case"] = "missing_completion_sticky_fault"
        record(result)
        channel.reset()
        result = channel.check_rate(0, 1)
        result["case"] = "fault_reset_recovery"
        record(result)

        # Reset while the producer clock is stopped. Both FIFO ends must stay
        # reset until the stopped clock resumes; no old code may be replayed.
        channel.write("pause", 1)
        channel.write("reset", 1)
        channel.write("enable", 0)
        time.sleep(0.01)
        channel.write("reset", 0)
        time.sleep(0.01)
        result = channel.measure()
        assert result["issued"] == result["completed"] == result["steps"] == 0, result
        result["case"] = "reset_with_stopped_producer"
        record(result)
        channel.write("pause", 0)
        time.sleep(0.01)
        result = channel.measure()
        assert result["issued"] == result["completed"] == result["steps"] == 0, result
        assert channel.read("backend_status") == 0
        result["case"] = "resume_without_stale_commands"
        record(result)
        result = channel.check_rate(65535, 1)
        result["case"] = "fresh_command_after_resume"
        record(result)
        channel.write("enable", 0)
        channel.write("reset", 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csr-csv", required=True,               help="CSR CSV from the loaded test image.")
    parser.add_argument("--host",    default="localhost",         help="LiteX server host.")
    parser.add_argument("--port",    default=1234, type=int,      help="LiteX server port.")
    parser.add_argument("--output",  default="mmcm-results.json", help="Measurement log.")
    args = parser.parse_args()
    results = []

    def record(result):
        results.append(result)
        Path(args.output).write_text(json.dumps(results, indent=2) + "\n")
        print(json.dumps(result), flush=True)

    bus = RemoteClient(host=args.host, port=args.port, csr_csv=args.csr_csv,
        timeout=5, raise_on_timeout=True)
    bus.open()
    try:
        qualify(bus, record)
    finally:
        bus.close()
    print(f"PASS: {len(results)} physical MMCM measurements", flush=True)


if __name__ == "__main__":
    main()
