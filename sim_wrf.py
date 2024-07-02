#!/usr/bin/env python3

import sys
import argparse

from migen import *

from litex.gen import *

from litex.build.generic_platform import *
from litex.build.sim import SimPlatform

from litex.build.sim.config import SimConfig

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.interconnect import stream

from litex.soc.interconnect import stream

# IOs ----------------------------------------------------------------------------------------------

_io = [
    ("clk_sys",  0, Pins(1)),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(SimPlatform):
    default_clk_name = "clk_sys"
    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

# Sim ----------------------------------------------------------------------------------------------

class SimSoC(SoCCore):
    def __init__(self, sys_clk_freq=int(1e6)):
        # Platform ---------------------------------------------------------------------------------
        platform  = Platform()
        self.comb += platform.trace.eq(1) # Always enable tracing.

        # SoC --------------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq)

        # Sim Finish -------------------------------------------------------------------------------
        cycles = Signal(32)
        self.sync += cycles.eq(cycles + 1)
        self.sync += If(cycles == 10000, Finish())

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", action="store_true", help="Enable VCD tracing")
    args = parser.parse_args()

    # Simulation configuration.
    sim_config = SimConfig(default_clk="clk_sys")

    # Setup and run the simulation.
    soc = SimSoC()
    builder = Builder(soc, output_dir="build")
    builder.build(sim_config=sim_config, trace=args.trace)

if __name__ == "__main__":
    main()