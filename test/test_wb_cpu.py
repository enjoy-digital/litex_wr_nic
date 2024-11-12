#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import sys
import time
import argparse
from tqdm import tqdm
from litex import RemoteClient

# Constants ----------------------------------------------------------------------------------------

CPU_RST_REG = 0x0
CPU_ADR_REG = 0x4
CPU_DAT_REG = 0x8
DEFAULT_DUMP_LENGTH = 128 * 1024  # Default dump length: 128 KB

# CPU ----------------------------------------------------------------------------------------------

class CPU:
    def __init__(self, bus):
        self.bus = bus

    def write_reg(self, addr, dat):
        self.bus.write(self.bus.mems.wr_wb_slave.base + 0x20b00 + addr, dat)

    def read_reg(self, addr):
        return self.bus.read(self.bus.mems.wr_wb_slave.base + 0x20b00 + addr)

    def reset(self, value):
        self.write_reg(CPU_RST_REG, value)

    def ram_read(self, addr):
        self.write_reg(CPU_ADR_REG, addr)
        data = self.read_reg(CPU_DAT_REG)
        return data

    def ram_write(self, addr, data):
        self.write_reg(CPU_ADR_REG, addr)
        self.write_reg(CPU_DAT_REG, data)

    def load_firmware(self, firmware_data):
        """Load firmware binary data into CPU RAM in 32-bit."""
        self.reset(1)  # Hold CPU in reset
        for i in tqdm(range(0, len(firmware_data), 4), desc="Loading firmware", unit="word"):
            word = int.from_bytes(firmware_data[i:i+4], byteorder='big')
            self.ram_write(i // 4, word)
            time.sleep(1e-4) # CHECKME: Required over JTAG.
        self.reset(0)  # Release CPU reset

    def dump_firmware(self, length):
        """Dump CPU RAM contents up to specified length in 32-bit words."""
        self.reset(1)  # Hold CPU in reset
        data = []
        for i in tqdm(range(0, length, 4), desc="Dumping firmware", unit="word"):
            word = self.ram_read(i // 4)
            data.extend(word.to_bytes(4, byteorder='big'))
        self.reset(0)  # Release CPU reset
        return data

# Utilities ----------------------------------------------------------------------------------------

def write_firmware_file(filename, data):
    with open(filename, "wb") as f:
        f.write(bytes(data))

def read_firmware_file(filename):
    with open(filename, "rb") as f:
        return list(f.read())

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Control CPU on White Rabbit Core via Etherbone.")
    parser.add_argument("--reset", action="store_true", help="Manually reset the CPU.")
    parser.add_argument("--build-firmware", action="store_true", help="Build CPU firmware.")
    parser.add_argument("--load-firmware", metavar="FILE", help="Path to firmware binary for loading.")
    parser.add_argument("--dump-firmware", metavar="FILE", help="Filename to save dumped firmware.")
    parser.add_argument("--dump-length", type=int, default=DEFAULT_DUMP_LENGTH, help="Dump length in bytes.")
    args = parser.parse_args()

    bus = RemoteClient()
    bus.open()

    cpu = CPU(bus=bus)

    # Reset command
    if args.reset:
        print("Resetting CPU manually...")
        cpu.reset(1)
        cpu.reset(0)

    # Build firmware
    if args.build_firmware:
        os.system("cd ../firmware && ./build.py")

    # Load firmware command
    if args.load_firmware:
        firmware_data = read_firmware_file(args.load_firmware)
        print(f"Loading firmware from {args.load_firmware}...")
        cpu.load_firmware(firmware_data)

    # Dump firmware command
    if args.dump_firmware:
        print(f"Dumping firmware to {args.dump_firmware}...")
        firmware_data = cpu.dump_firmware(args.dump_length)
        write_firmware_file(args.dump_firmware, firmware_data)

    # Close the bus connection
    bus.close()

if __name__ == "__main__":
    main()
