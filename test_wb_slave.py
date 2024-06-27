#!/usr/bin/env python3

from litex import RemoteClient

# Define the memory map regions
memory_map = {
    "Minic"                       : 0x000,
    "Endpoint"                    : 0x100,
    "Softpll"                     : 0x200,
    "PPS gen"                     : 0x300,
    "Syscon"                      : 0x400,
    "UART"                        : 0x500,
    "OneWire"                     : 0x600,
    "WRPC diagnostics (user)"     : 0x800,
    "WRPC diagnostics (firmware)" : 0x900,
    "Freq monitor"                : 0xa00,
    "CPU CSR"                     : 0xb00,
    "Secbar SDB"                  : 0xc00,
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
