#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litex import RemoteClient

# Define the memory map regions
memory_map = {
    "Minic"                       : 0x20000,
    "Endpoint"                    : 0x20100,
    "Softpll"                     : 0x20200,
    "PPS gen"                     : 0x20300,
    "Syscon"                      : 0x20400,
    "UART"                        : 0x20500,
    "OneWire"                     : 0x20600,
    "WRPC diagnostics (user)"     : 0x20800,
    "WRPC diagnostics (firmware)" : 0x20900,
    "Freq monitor"                : 0x20a00,
    "CPU CSR"                     : 0x20b00,
    "Secbar SDB"                  : 0x20c00,
}

def main():
    # Connect to the LiteX server
    client = RemoteClient()
    client.open()

    # Read and print values from each region
    print("Dumping MMAP ranges:")
    for component, base_address in memory_map.items():
        print(f"Component: {component}")
        for offset in range(0x0, 0x100, 0x4):
            address = 0x2000_0000 + base_address + offset
            value = client.read(address)  # Read 32-bit value from the address
            print(f"  Address 0x{address:04x}: 0x{value:08x}")

    # Close the client connection
    client.close()

if __name__ == "__main__":
    main()
