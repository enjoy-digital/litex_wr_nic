#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import time
import argparse

from litex import RemoteClient

# LMX2572 Class ------------------------------------------------------------------------------------

class LMX2572:
    def __init__(self, bus):
        self.bus = bus

    def write_reg(self, addr, value):
        """Writes a 24-bit word to the LMX2572.
        Constructs the word: 1-bit Write/Read, 7-bit Address, 16-bit Data."""
        data = (0 << 23) | (addr << 16) | (value & 0xFFFF)  # 0 for Write, addr is 7 bits, value is 16 bits.
        # Write MOSI.
        self.bus.regs.lmx2572_mosi.write(data)
        # Start Transfer.
        self.bus.regs.lmx2572_control.write((24 << 8) | 0b1)
        # Wait Done.
        while (self.bus.regs.lmx2572_status.read() & 0b1) != 0b1:
            pass
        print(f"Reg Write @0x{addr:02X}: 0x{value:04X}")

    def load(self, config_file):
        """Loads a configuration from a file and writes it to the LMX2572."""
        print(f"Loading configuration from: {config_file}")
        with open(config_file, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):  # Skip empty lines and comments.
                    continue
                try:
                    # Extract the hex data (everything after the tab or space).
                    data = int(line.split()[1], 16)
                    addr = (data >> 16) & 0x7F  # Extract the 7-bit address.
                    value = data & 0xFFFF       # Extract the 16-bit data.
                    self.write_reg(addr, value)
                    time.sleep(0.01)  # Small delay between writes.
                except Exception as e:
                    print(f"Failed to write line: {line} ({e})")

    def toggle_sync(self):
        self.bus.regs.main_lmx2572_sync.write(0)
        self.bus.regs.main_lmx2572_sync.write(1)

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Control LMX2572 via Etherbone.")
    parser.add_argument("--config",    type=str, help="Path to configuration file.")
    parser.add_argument("--write-reg", nargs=2,  help="Write to a register (address and value in hex).")
    args = parser.parse_args()

    # Open the bus connection.
    bus = RemoteClient()
    bus.open()

    # Initialize LMX2572 controller.
    lmx2572 = LMX2572(bus)

    # Load configuration from a file.
    if args.config:
        lmx2572.load(args.config)
        for i in range(4):
            lmx2572.toggle_sync()

    # Write a single register if specified.
    if args.write_reg:
        try:
            addr = int(args.write_reg[0], 16)
            value = int(args.write_reg[1], 16)
            lmx2572.write_reg(addr, value)
        except ValueError:
            print("Invalid address or value format. Use hex values (e.g., 0x01 0x1234).")

    # Close the bus connection.
    bus.close()

if __name__ == "__main__":
    main()
