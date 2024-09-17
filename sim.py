#!/usr/bin/env python3

import argparse

from migen import *

from litex.tools.litex_sim import *

# WRCSim --------------------------------------------------------------------------------------------

class WRCSim(SimSoC):
    def __init__(self):
        SimSoC.__init__(self, cpu_type="None", with_uart=False)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="White Rabbit Core Simulation")
    parser.add_argument("--trace",       action="store_true", help="Enable Tracing")
    parser.add_argument("--trace-fst",   action="store_true", help="Enable FST tracing (default=VCD)")
    parser.add_argument("--trace-start", default=0,           help="Cycle to start tracing")
    parser.add_argument("--trace-end",   default=-1,          help="Cycle to end tracing")
    args = parser.parse_args()

    sim_config = SimConfig(default_clk="sys_clk")

    # SoC ------------------------------------------------------------------------------------------
    soc = WRCSim()

    # Build/Run ------------------------------------------------------------------------------------
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(sim_config=sim_config,
        trace       = args.trace,
        trace_fst   = args.trace,
        trace_start = int(args.trace_start),
        trace_end   = int(args.trace_end)
    )

if __name__ == "__main__":
    main()
