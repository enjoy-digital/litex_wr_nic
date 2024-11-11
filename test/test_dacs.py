#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse
import time
from litex import RemoteClient

# DAC control functions ----------------------------------------------------------------------------

def set_dac(bus, dac_value, dac_load, value):
    """Sets a DAC value and loads it."""
    dac_value.write(value)
    dac_load.write(1)
    dac_load.write(0)

def ramp_dac(bus, dac_value, dac_load):
    """Continuously ramps DAC from 0 to 65535 in 1 second, updating every 10 ms, restarting at 0 when done."""
    step = 65535 // 100  # 100 steps for a 1-second ramp with 10 ms updates
    try:
        while True:
            for value in range(0, 65536, step):
                set_dac(bus, dac_value, dac_load, value)
                time.sleep(0.01)  # 10 ms delay
    except KeyboardInterrupt:
        print("Ramp interrupted by user.")

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Control DACs on White Rabbit Core via Etherbone.")
    parser.add_argument("--force",           action="store_true", help="Force DAC Control.")
    parser.add_argument("--unforce",         action="store_true", help="Unforce DAC Control.")
    parser.add_argument("--refclk-dac",      type=int,            help="Set value for RefClk DAC.")
    parser.add_argument("--dmtd-dac",        type=int,            help="Set value for DMTD DAC.")
    parser.add_argument("--refclk-dac-ramp", action="store_true", help="Enable continuous ramp for RefClk DAC.")
    parser.add_argument("--dmtd-dac-ramp",   action="store_true", help="Enable continuous ramp for DMTD DAC.")
    args = parser.parse_args()

    # Open the bus connection.
    bus = RemoteClient()
    bus.open()

    # Force.
    if args.force:
        bus.regs.refclk_dac_force.write(1)
        bus.regs.dmtd_dac_force.write(1)

    # Un-Force.
    if args.unforce:
        bus.regs.refclk_dac_force.write(0)
        bus.regs.dmtd_dac_force.write(0)

    # Set the refclk DAC if specified.
    if args.refclk_dac is not None:
        set_dac(bus, bus.regs.refclk_dac_value, bus.regs.refclk_dac_load, args.refclk_dac)

    # Set the dmtd DAC if specified.
    if args.dmtd_dac is not None:
        set_dac(bus, bus.regs.dmtd_dac_value, bus.regs.dmtd_dac_load, args.dmtd_dac)

    # Start refclk DAC ramp if specified.
    if args.refclk_dac_ramp:
        print("Starting continuous ramp for refclk DAC.")
        ramp_dac(bus, bus.regs.refclk_dac_value, bus.regs.refclk_dac_load)

    # Start dmtd DAC ramp if specified.
    if args.dmtd_dac_ramp:
        print("Starting continuous ramp for dmtd DAC.")
        ramp_dac(bus, bus.regs.dmtd_dac_value, bus.regs.dmtd_dac_load)

    # Close the bus connection.
    bus.close()

if __name__ == "__main__":
    main()
