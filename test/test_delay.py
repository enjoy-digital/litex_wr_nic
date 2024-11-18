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

# Delay Line Control Functions ---------------------------------------------------------------------

def set_delay_line(bus, channel, value):
    """
    Set a value on the selected delay line channel and wait for the operation to complete.

    :param bus: RemoteClient object.
    :param channel: Channel to configure (0 or 1).
    :param value: Value to set (0 to 511).
    """
    if value < 0 or value > 511:
        raise ValueError("Value must be between 0 and 511.")
    if channel not in [0, 1]:
        raise ValueError("Channel must be 0 or 1.")

    print(f"Setting delay line: channel={channel}, value={value}")
    bus.regs.delay_line_sel.write(channel)
    bus.regs.delay_line_value.write(value)
    bus.regs.delay_line_req.write(1)

    # Wait for the delay line to finish processing.
    while bus.regs.delay_line_busy.read():
        time.sleep(0.01)  # Poll every 10 ms
    print("Delay line set successfully.")

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Control Delay Line Module via Etherbone.")
    parser.add_argument("--channel", type=int, choices=[0, 1], required=True, help="Select delay line channel (0 or 1).")
    parser.add_argument("--value",   type=int, required=True,                 help="Set value for selected delay line channel (0-511).")
    args = parser.parse_args()

    # Open the bus connection.
    bus = RemoteClient()
    bus.open()

    try:
        # Set delay line with the specified channel and value.
        set_delay_line(bus, args.channel, args.value)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Close the bus connection.
        bus.close()

if __name__ == "__main__":
    main()
