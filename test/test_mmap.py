#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse
from litex import RemoteClient

# Constants ----------------------------------------------------------------------------------------

BASE_ADDRESS = 0x2000_0000
REGION_SIZE  = 0x100  # Each region spans 256 bytes (0x100)
WORD_SIZE    = 4      # 4 bytes per word

MEMORY_MAP = {
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

# Memory Reader ------------------------------------------------------------------------------------

class MemoryReader:
    def __init__(self, client):
        self.client = client

    def read_region(self, name, base_address):
        """Reads and dumps memory for a given region."""
        print(f"Dumping region: {name}")
        for offset in range(0, REGION_SIZE, WORD_SIZE):
            address = BASE_ADDRESS + base_address + offset
            value   = self.client.read(address)
            print(f"  Address 0x{address:08x}: 0x{value:08x}")

# Utilities ----------------------------------------------------------------------------------------

def list_regions():
    """List all available memory regions."""
    print("Available regions:")
    for name, base_address in MEMORY_MAP.items():
        print(f"  {name:30} Base Address: 0x{BASE_ADDRESS + base_address:08x}")

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Dump memory-mapped regions via Etherbone.")
    parser.add_argument("--list",   action="store_true", help="List available memory regions.")
    parser.add_argument("--region", type=str,            help="Specific region to dump (default: all).")
    args = parser.parse_args()

    # List Regions.
    if args.list:
        list_regions()
        return

    # Connect to the LiteX server
    client = RemoteClient()
    client.open()

    reader = MemoryReader(client)

    # Dump specific or all regions
    if args.region:
        region_name = args.region
        if region_name in MEMORY_MAP:
            reader.read_region(region_name, MEMORY_MAP[region_name])
        else:
            print(f"Error: Region '{region_name}' not found in memory map.")
    else:
        print("Dumping all regions...")
        for name, base_address in MEMORY_MAP.items():
            reader.read_region(name, base_address)

    # Close the client connection
    client.close()

if __name__ == "__main__":
    main()
