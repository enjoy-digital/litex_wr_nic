#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Manage White Rabbit through a LiteX RemoteClient connection."""

import sys
import json
import time
import select
import termios
import argparse
import contextlib
from pathlib import Path

from litex import RemoteClient

from litex_wr_nic.wr_boot import WR_BOOT_MAGIC, inspect_boot_image

# Constants ----------------------------------------------------------------------------------------

WR_INFO_MAGIC = 0x57524331
CPU_CSR       = 0x20b00
CPU_RESET     = 0x00
CPU_ADDRESS   = 0x04
CPU_DATA      = 0x08
CPU_HALTED    = 0x80
CPU_HALT      = 0x84
CPU_TYPES     = {0: "urv", 1: "vexriscv"}

# Management ---------------------------------------------------------------------------------------

class WRClient:
    def __init__(self, bus, cpu_type=None, wr_region="wr_wb_slave", memory_region="wr_cpu_mem",
        timeout=5.0):
        self.bus     = bus
        self.timeout = timeout
        self.region  = getattr(bus.mems, wr_region)
        self.memory  = getattr(bus.mems, memory_region, None)
        self.regs    = vars(bus.regs)
        info = [name[:-6] for name in self.regs if name.endswith("wr_info_magic")]
        if len(info) > 1:
            raise ValueError("Multiple WR information banks found; use a CSR map for one core.")
        self.info = info[0] if info else None
        if self.info:
            if self.reg("magic").read() != WR_INFO_MAGIC:
                raise RuntimeError("WR information signature mismatch; check the bitstream and csr.csv.")
            detected = CPU_TYPES.get(self.reg("cpu_type").read())
            if detected is None:
                raise RuntimeError("Unsupported WR CPU identifier.")
            external = self.reg("memory_mode").read()
            if bool(external) != (self.memory is not None):
                raise RuntimeError("WR memory map disagrees with hardware; check --memory-region and csr.csv.")
            if cpu_type and cpu_type != detected:
                raise ValueError("Requested CPU type disagrees with hardware.")
            cpu_type = detected
        elif cpu_type is None and self.memory is None:
            cpu_type = "urv"
        self.host_ready = next((reg for name, reg in self.regs.items()
            if name.endswith("wr_cpu_boot_host_ready")), None)
        self.memory_ready = next((reg for name, reg in self.regs.items()
            if name.endswith("wr_cpu_boot_memory_ready")), None)
        self.cpu_type = cpu_type
        self.size     = min(self.memory.size if self.memory else 128*1024, 128*1024)

    def reg(self, name):
        return self.regs[self.info + "_" + name]

    def read_cpu(self, offset):
        return self.bus.read(self.region.base + CPU_CSR + offset)

    def write_cpu(self, offset, value):
        self.bus.write(self.region.base + CPU_CSR + offset, value)

    def wait(self, predicate, message):
        deadline = time.monotonic() + self.timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(message)
            time.sleep(0.01)

    def stop(self):
        if self.cpu_type not in CPU_TYPES.values():
            raise ValueError("This older image needs --cpu-type urv or --cpu-type vexriscv.")
        if self.read_cpu(CPU_RESET) & 1:
            return
        if self.cpu_type == "urv" and not (self.host_ready and not self.host_ready.read()):
            # Drain the pipeline before reset. Direct early resets on the
            # upstream private-memory uRV wrapper can otherwise hang.
            previous = self.read_cpu(CPU_HALT)
            self.write_cpu(CPU_HALT, 1)
            try:
                self.wait(lambda: self.read_cpu(CPU_HALTED) & 1,
                    "uRV did not halt; CPU reset was not asserted.")
            except Exception:
                self.write_cpu(CPU_HALT, previous)
                raise
        self.write_cpu(CPU_RESET, 1)
        if not self.read_cpu(CPU_RESET) & 1:
            raise RuntimeError("CPU reset assertion did not read back.")
        if self.cpu_type == "urv":
            self.write_cpu(CPU_HALT, 0)
        time.sleep(0.05)

    def start(self):
        if self.host_ready:
            if not self.memory_ready.read():
                raise RuntimeError("The memory controller is not ready; CPU remains in reset.")
            self.host_ready.write(1)
            if self.info:
                self.wait(lambda: self.reg("status").read() & 1, "Host boot readiness did not reach the CPU.")
        if self.info and not self.reg("status").read() & 1:
            raise RuntimeError("WR memory is not ready; CPU remains in reset.")
        self.write_cpu(CPU_RESET, 0)
        if self.read_cpu(CPU_RESET) & 1:
            raise RuntimeError("CPU reset release did not read back.")

    def restart(self):
        self.stop()
        self.start()

    def read_word(self, offset):
        if self.memory:
            return self.bus.read(self.memory.base + offset)
        self.write_cpu(CPU_ADDRESS, offset // 4)
        return self.read_cpu(CPU_DATA)

    def write_word(self, offset, value):
        if self.memory:
            self.bus.write(self.memory.base + offset, value)
        else:
            self.write_cpu(CPU_ADDRESS, offset // 4)
            self.write_cpu(CPU_DATA, value)

    def load_firmware(self, data, firmware_cpu=None):
        if data.startswith(WR_BOOT_MAGIC):
            image = inspect_boot_image(data, max_payload=self.size, cpu_type=self.cpu_type)
            if firmware_cpu and image["cpu_type"] and firmware_cpu != image["cpu_type"]:
                raise ValueError("Declared firmware CPU disagrees with boot image metadata.")
            firmware_cpu = image["cpu_type"] or firmware_cpu
            data = image["payload"]
        if firmware_cpu is None:
            raise ValueError("Raw and legacy images require --firmware-cpu.")
        if firmware_cpu != self.cpu_type:
            raise ValueError("Firmware CPU profile does not match the loaded hardware.")
        if not data or len(data) > self.size:
            raise ValueError("Firmware must fit in the reserved WR CPU memory.")
        if self.memory_ready and not self.memory_ready.read():
            raise RuntimeError("The memory controller is not ready for firmware upload.")
        data  = data.ljust(self.size, b"\x00")
        order = "little" if self.memory else "big"
        self.stop()
        # Use the shared SoC memory path, including any HyperRAM cache.
        words = [int.from_bytes(data[offset:offset+4], order) for offset in range(0, len(data), 4)]
        if self.memory:
            for index in range(0, len(words), 16):
                chunk = words[index:index+16]
                self.bus.write(self.memory.base + 4*index, chunk)
                # RemoteClient writes have no response. Fence each batch so a
                # slow transport cannot accumulate the entire image ahead of
                # the first read and exceed its response timeout.
                self.bus.read(self.memory.base + 4*(index + len(chunk) - 1))
            for index in range(0, len(words), 64):
                expected = words[index:index+64]
                actual = self.bus.read(self.memory.base + 4*index, length=len(expected))
                if actual != expected:
                    raise RuntimeError(f"Firmware verification failed near 0x{4*index:08x}; CPU remains in reset.")
        else:
            for index, word in enumerate(words):
                self.write_word(4*index, word)
            for index, word in enumerate(words):
                if self.read_word(4*index) != word:
                    raise RuntimeError(f"Firmware verification failed at 0x{4*index:08x}; CPU remains in reset.")
        self.start()

    def status(self):
        result = dict(
            cpu_type        = self.cpu_type or "unknown (use --cpu-type)",
            memory          = "soc" if self.memory else "private",
            memory_size     = self.size,
            reset_requested = bool(self.read_cpu(CPU_RESET) & 1),
        )
        if self.info:
            status = self.reg("status").read()
            result.update(
                memory_ready         = bool(status & 1),
                link_up              = bool(status & 4),
                time_valid           = bool(status & 8),
                reset_reason         = "host CPU reset" if self.reg("reset_reason").read() else "system reset",
                reset_count          = self.reg("reset_count").read(),
                build_firmware_sha256 = f"{self.reg('firmware_hash').read():064x}",
            )
        for name, reg in self.regs.items():
            if ("wr_cpu_boot_" in name or "wr_cpu_bridge_" in name) and reg.mode == "ro":
                result[name] = reg.read()
        return result

# Console ------------------------------------------------------------------------------------------

class WRConsole:
    def __init__(self, bus, prefix=None, timeout=5.0):
        self.bus     = bus
        self.timeout = timeout
        regs       = vars(bus.regs)
        candidates = [name[:-5] for name in regs if name.endswith("xover_rxtx")]
        if prefix is None:
            if len(candidates) != 1:
                raise ValueError("Select the WR crossover UART with --uart-prefix.")
            prefix = candidates[0]
        self.data  = regs[prefix + "_rxtx"]
        self.empty = regs[prefix + "_rxempty"]
        self.full  = regs[prefix + "_txfull"]
        parent = prefix.rsplit("_xover", 1)[0]
        self.control  = regs.get(parent + "_control")
        self.level    = regs.get(parent + "_rxlevel")
        self.overflow = regs.get(parent + "_rxoverflow")

    @contextlib.contextmanager
    def selected(self):
        previous = self.control.read() if self.control else None
        if self.control:
            self.control.write(1) # Crossover, manual mode.
        try:
            yield self
        finally:
            if self.control:
                self.control.write(previous)

    def receive(self, limit=128):
        if self.overflow and self.overflow.read():
            raise RuntimeError(
                "WR console RX overflow; output was lost. "
                "Reload the FPGA or increase crossover_rx_depth.")
        if self.level:
            count = min(limit, self.level.read())
            if count:
                return bytes(word & 0xff for word in self.bus.read(
                    self.data.addr, length=count, burst="fixed"))
            return b""
        data = bytearray()
        for _ in range(limit):
            if self.empty.read():
                break
            data.append(self.data.read() & 0xff)
        return bytes(data)

    def send(self, data):
        for byte in data:
            deadline = time.monotonic() + self.timeout
            while self.full.read():
                if time.monotonic() >= deadline:
                    raise TimeoutError("WR console TX FIFO is full.")
                time.sleep(0.001)
            self.data.write(byte)
            time.sleep(0.03) # WRPC polls its UART; pace input like typing.

    def command(self, command, timeout=60.0):
        self.receive()
        self.send(command.encode("ascii") + b"\r")
        deadline = time.monotonic() + timeout
        output   = bytearray()
        while time.monotonic() < deadline:
            output.extend(self.receive())
            if b"wrc#" in output:
                return output.decode("utf-8", errors="replace")
            time.sleep(0.01)
        raise TimeoutError("WR console prompt timed out: " + repr(bytes(output[-500:])))

    def interactive(self):
        import tty
        old = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
        try:
            if old:
                tty.setcbreak(sys.stdin.fileno())
            while True:
                data = self.receive()
                if data:
                    sys.stdout.buffer.write(data)
                    sys.stdout.buffer.flush()
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    data = sys.stdin.buffer.read(1)
                    if not data or data == b"\x1d": # Ctrl-].
                        break
                    self.send(b"\r" if data == b"\n" else data)
        finally:
            if old:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)

# Command Line -------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    # Remote connection.
    parser.add_argument("--host",    default="localhost")
    parser.add_argument("--port",    default=1234, type=int)
    parser.add_argument("--csr-csv", default="csr.csv")

    # WR integration.
    parser.add_argument("--cpu-type",      choices=tuple(CPU_TYPES.values()))
    parser.add_argument("--wr-region",     default="wr_wb_slave")
    parser.add_argument("--memory-region", default="wr_cpu_mem")
    parser.add_argument("--uart-prefix")

    # Commands.
    commands = parser.add_subparsers(dest="command", required=True)
    for command in ("status", "diagnose", "restart"):
        commands.add_parser(command)
    console = commands.add_parser("console")
    console.add_argument("--command", dest="console_command")
    load = commands.add_parser("load-firmware")
    load.add_argument("file", type=Path)
    load.add_argument("--firmware-cpu", choices=tuple(CPU_TYPES.values()), help="Required for raw/legacy images.")
    args = parser.parse_args()
    bus = RemoteClient(
        host             = args.host,
        port             = args.port,
        csr_csv          = args.csr_csv,
        timeout          = 5,
        raise_on_timeout = True,
    )
    bus.open()
    try:
        if args.command == "console":
            with WRConsole(bus, prefix=args.uart_prefix).selected() as console:
                if args.console_command:
                    print(console.command(args.console_command), end="")
                else:
                    console.interactive()
        else:
            wr = WRClient(bus,
                cpu_type      = args.cpu_type,
                wr_region     = args.wr_region,
                memory_region = args.memory_region,
            )
            if args.command == "restart":
                wr.restart()
            elif args.command == "load-firmware":
                wr.load_firmware(args.file.read_bytes(), args.firmware_cpu)
            else:
                result = wr.status()
                if args.command == "diagnose":
                    with WRConsole(bus, prefix=args.uart_prefix).selected() as console:
                        result["console"] = {
                            command: console.command(command)
                            for command in ("ver", "uptime", "time")
                        }
                print(json.dumps(result, indent=2))
    finally:
        bus.close()


if __name__ == "__main__":
    main()
