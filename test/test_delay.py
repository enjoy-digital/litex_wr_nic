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

def set_delay_line(bus, channel, fine_value, coarse_value):
    """
    Set fine and coarse delay values for the selected channel and wait for the operation to complete.
    """
    if fine_value < 0 or fine_value > 511:
        raise ValueError("Fine value must be between 0 and 511.")
    if coarse_value < 0 or coarse_value > 7:
        raise ValueError("Coarse value must be between 0 and 7.")
    if channel not in [0, 1]:
        raise ValueError("Channel must be 0 (clk10_out) or 1 (pps_out).")

    # Set the coarse delay value.
    if channel == 0:
        print(f"Setting coarse delay for clk10_out: {coarse_value}")
        bus.regs.clk10_out_coarse_delay_value.write(coarse_value)
    elif channel == 1:
        print(f"Setting coarse delay for pps_out: {coarse_value}")
        bus.regs.pps_out_coarse_delay_value.write(coarse_value)

    # Set the fine delay value.
    print(f"Setting fine delay for channel={channel}, value={fine_value}")
    bus.regs.fine_delay_channel.write(channel)
    bus.regs.fine_delay_value.write(fine_value)

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Control Delay Line Module via Etherbone.")
    parser.add_argument("--channel", type=int, choices=[0, 1], required=True, help="Select delay line channel (0 for clk10_out, 1 for pps_out).")
    parser.add_argument("--fine",    type=int, required=True, help="Set fine delay value (0-511).")
    parser.add_argument("--coarse",  type=int, required=True, help="Set coarse delay value (0-7).")
    args = parser.parse_args()

    # Open the bus connection.
    bus = RemoteClient()
    bus.open()

    try:
        # Set delay line with the specified channel, fine, and coarse values.
        set_delay_line(bus, args.channel, args.fine, args.coarse)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Close the bus connection.
        bus.close()

if __name__ == "__main__":
    main()
