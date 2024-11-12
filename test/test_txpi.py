#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse
from litex import RemoteClient

# TX PI control functions --------------------------------------------------------------------------

def set_txpi(bus, enable, value):
    """Sets the TX PI control values based on enable and value parameters."""
    # Determine the sign and the absolute value.
    sign = 1 if value < 0 else 0
    abs_value = abs(value) & 0xF  # Limit value to 4 bits.

    # Set the CSR storages.
    bus.regs.tx_pi_control_enable.write(enable)
    bus.regs.tx_pi_control_sign.write(sign)
    bus.regs.tx_pi_control_value.write(abs_value)

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Control TX PI (Phase Interpolator) on FPGA via Etherbone.")
    parser.add_argument("--enable", type=int, choices=[0, 1], required=True, help="Enable or disable TX PI (0 or 1).")
    parser.add_argument("--value",  type=int, required=True, help="4-bit signed value for TX PI adjustment (-8 to 7).")
    args = parser.parse_args()

    # Open the bus connection.
    bus = RemoteClient()
    bus.open()

    # Set TX PI control based on parsed arguments.
    set_txpi(bus, args.enable, args.value)

    # Close the bus connection.
    bus.close()

if __name__ == "__main__":
    main()
