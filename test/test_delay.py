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

SMA_MAP = {
    "clk10m_out" : 0,
    "pps_out"    : 1,
}

# Macro Delay Configuration ------------------------------------------------------------------------

def set_macro_delay(bus, channel, macro_value):
    if macro_value < 1:
        raise ValueError("Macro delay value must be at least 1.")
    if channel == 0:
        print(f"Setting macro delay for clk10m_out to {macro_value}")
        bus.regs.clk10m_macro_delay_value.write(macro_value)
    elif channel == 1:
        print(f"Setting macro delay for pps_out to {macro_value}")
        bus.regs.pps_macro_delay_value.write(macro_value)

# Coarse Delay Configuration -----------------------------------------------------------------------

def set_coarse_delay(bus, channel, coarse_value):
    if coarse_value < 0 or coarse_value > 63:
        raise ValueError("Coarse value must be between 0 and 63.")
    if channel == 0:
        print(f"Setting coarse delay for clk10m_out to {coarse_value}")
        bus.regs.clk10m_out_coarse_delay_value.write(coarse_value)
    elif channel == 1:
        print(f"Setting coarse delay for pps_out to {coarse_value}")
        bus.regs.pps_out_coarse_delay_value.write(coarse_value)

# Fine Delay Configuration -------------------------------------------------------------------------

def set_fine_delay(bus, channel, fine_value):
    if fine_value < 0 or fine_value > 511:
        raise ValueError("Fine value must be between 0 and 511.")
    print(f"Setting fine delay for channel {channel} to {fine_value}")
    bus.regs.fine_delay_channel.write(channel)
    bus.regs.fine_delay_value.write(fine_value)

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Control SyncOut Delays via JTAGBone/Etherbone.")
    parser.add_argument("--sma",   type=str, choices=["clk10m_out", "pps_out"], help="Select SMA output (clk10m_out or pps_out).")
    parser.add_argument("--macro", type=int, help="Set macro delay value (in clock cycles).")
    parser.add_argument("--coarse",type=int, help="Set coarse delay value (0-63).")
    parser.add_argument("--fine",  type=int, help="Set fine delay value (0-511).")

    args = parser.parse_args()

    if args.sma is None:
        raise ValueError("--sma is required to select the SMA output.")

    # Map SMA to channel.
    channel = SMA_MAP[args.sma]

    # Open the bus connection.
    bus = RemoteClient()
    bus.open()

    try:
        # Configure macro delay.
        if args.macro is not None:
            set_macro_delay(bus, channel, args.macro)

        # Configure coarse delay.
        if args.coarse is not None:
            set_coarse_delay(bus, channel, args.coarse)

        # Configure fine delay.
        if args.fine is not None:
            set_fine_delay(bus, channel, args.fine)

        if args.macro is None and args.coarse is None and args.fine is None:
            print("No operation specified. Use --macro, --coarse, or --fine with the appropriate arguments.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Close the bus connection.
        bus.close()

if __name__ == "__main__":
    main()
