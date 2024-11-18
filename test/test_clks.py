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

# Constants ----------------------------------------------------------------------------------------

NUM_CLOCKS = 5
CLOCK_MAPPING = {
    0: "Sys Clk",
    1: "DMTD Clk",
    2: "Clk Ref",
    3: "Clk10m Ext",
    4: "Clk62m5 Ext",
}

# Functions ----------------------------------------------------------------------------------------

def measure_clocks(bus, count, delay):
    """Measure and display clock frequencies."""
    print("Initializing measurements...")
    for i in range(NUM_CLOCKS):
        getattr(bus.regs, f"clk_measurement_clk{i}_latch").write(1)
    previous_values = [getattr(bus.regs, f"clk_measurement_clk{i}_value").read() for i in range(NUM_CLOCKS)]
    time.sleep(delay)  # Initial delay for stability
    start_time = time.time()

    for measurement in range(count):
        time.sleep(delay)

        # Latch and read current values
        for i in range(NUM_CLOCKS):
            getattr(bus.regs, f"clk_measurement_clk{i}_latch").write(1)

        current_values = [getattr(bus.regs, f"clk_measurement_clk{i}_value").read() for i in range(NUM_CLOCKS)]
        elapsed_time   = time.time() - start_time
        start_time     = time.time()

        # Skip the first measurement
        if measurement == 0:
            print(f"Skipping first measurement (stabilization).")
            previous_values = current_values
            continue

        print(f"Measurement {measurement}:")
        for clk_index in range(NUM_CLOCKS):
            delta_value = current_values[clk_index] - previous_values[clk_index]
            frequency_mhz = delta_value / (elapsed_time * 1e6)
            clk_name = CLOCK_MAPPING.get(clk_index, f"Clock {clk_index}")
            print(f"  {clk_name}: {frequency_mhz:.2f} MHz")
            previous_values[clk_index] = current_values[clk_index]

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Measure clock frequencies of the SoC via Etherbone.")
    parser.add_argument("--count", type=int,   default=10,  help="Number of measurements (default: 10).")
    parser.add_argument("--delay", type=float, default=1.0, help="Delay between measurements in seconds (default: 1s).")
    args = parser.parse_args()

    bus = RemoteClient()
    bus.open()

    try:
        measure_clocks(bus, args.count, args.delay)
    finally:
        bus.close()

if __name__ == "__main__":
    main()
