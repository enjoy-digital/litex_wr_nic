#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse

from litex import RemoteClient

# Macro Delay Configuration ------------------------------------------------------------------------

def set_macro_delay(bus, macro_value):
    if macro_value < 1:
        raise ValueError("Macro delay value must be at least 1.")
    print(f"Setting macro delay to {macro_value}")
    bus.regs.macro_delay_value.write(macro_value)

# Coarse Delay Configuration -----------------------------------------------------------------------

def set_coarse_delay(bus, channel, coarse_value):
    if coarse_value < 0 or coarse_value > 7:
        raise ValueError("Coarse value must be between 0 and 7.")
    if channel not in [0, 1]:
        raise ValueError("Channel must be 0 (clk10_out) or 1 (pps_out).")
    if channel == 0:
        print(f"Setting coarse delay for clk10_out to {coarse_value}")
        bus.regs.clk10_out_coarse_delay_value.write(coarse_value)
    elif channel == 1:
        print(f"Setting coarse delay for pps_out to {coarse_value}")
        bus.regs.pps_out_coarse_delay_value.write(coarse_value)

# Fine Delay Configuration -------------------------------------------------------------------------

def set_fine_delay(bus, channel, fine_value):
    if fine_value < 0 or fine_value > 511:
        raise ValueError("Fine value must be between 0 and 511.")
    if channel not in [0, 1]:
        raise ValueError("Channel must be 0 (clk10_out) or 1 (pps_out).")
    print(f"Setting fine delay for channel {channel} to {fine_value}")
    bus.regs.fine_delay_channel.write(channel)
    bus.regs.fine_delay_value.write(fine_value)

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Control Delay Line Module via Etherbone.")
    parser.add_argument("--channel", type=int, choices=[0, 1], help="Select delay line channel (0 for clk10_out, 1 for pps_out).")
    parser.add_argument("--macro",   type=int, help="Set macro delay value (in clock cycles).")
    parser.add_argument("--coarse",  type=int, help="Set coarse delay value (0-7).")
    parser.add_argument("--fine",    type=int, help="Set fine delay value (0-511).")

    args = parser.parse_args()

    # Open the bus connection.
    bus = RemoteClient()
    bus.open()

    try:
        # Configure macro delay.
        if args.macro is not None:
            set_macro_delay(bus, args.macro)

        # Configure coarse delay.
        if args.coarse is not None:
            if args.channel is None:
                raise ValueError("--channel is required when setting coarse delay.")
            set_coarse_delay(bus, args.channel, args.coarse)

        # Configure fine delay.
        if args.fine is not None:
            if args.channel is None:
                raise ValueError("--channel is required when setting fine delay.")
            set_fine_delay(bus, args.channel, args.fine)

    except Exception as e:
        print(f"Error: {e}")
    finally:

        # Close the bus connection.
        bus.close()

if __name__ == "__main__":
    main()
