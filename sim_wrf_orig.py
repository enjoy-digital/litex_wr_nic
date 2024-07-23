#!/usr/bin/env python3

import sys
import argparse

from migen import *

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.build.generic_platform import *
from litex.build.sim import SimPlatform

from litex.build.sim.config import SimConfig

from litex.soc.interconnect import wishbone
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.interconnect import stream

from litex.soc.interconnect import stream

# IOs ----------------------------------------------------------------------------------------------

_io = [
    ("clk_sys",  0, Pins(1)),
    ("clk_wr",   0, Pins(1)),
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

        # Clocking ---------------------------------------------------------------------------------
        self.cd_sys = ClockDomain()
        self.cd_wr  = ClockDomain()
        self.comb += self.cd_sys.clk.eq( platform.request("clk_sys"))
        self.comb += self.cd_wr.clk.eq(platform.request("clk_wr"))

        # WRF Sim ----------------------------------------------------------------------------------

        wrf_bus = wishbone.Interface()

        wrf_snk_timer = WaitTimer(1000)
        self.submodules += wrf_snk_timer
        self.comb += wrf_snk_timer.wait.eq(~wrf_snk_timer.done)

        self.specials += Instance("wrf_snk_test",
            i_wr_sys_clk    = ClockSignal("sys"),
            i_u_senddata    = wrf_snk_timer.done,
            o_wrf_snk_adr   = wrf_bus.adr,
            o_wrf_snk_dat   = wrf_bus.dat_w,
            o_wrf_snk_cyc   = wrf_bus.stb,
            o_wrf_snk_stb   = wrf_bus.cyc,
            i_wrf_snk_ack   = wrf_bus.ack,
            i_wrf_snk_stall = 0,
            o_wrf_snk_we    = wrf_bus.we,
            o_wrf_snk_sel   = wrf_bus.sel,
        )
        self.platform.add_source("gateware/wrf_snk_test_orig.v")

        self.comb += wrf_bus.ack.eq(1)

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
    sim_config = SimConfig()
    sim_config.add_clocker(clk="clk_sys",  freq_hz=int(125e6))
    sim_config.add_clocker(clk="clk_wr",   freq_hz=int(62.5e6))

    # Setup and run the simulation.
    soc = SimSoC()
    builder = Builder(soc, output_dir="build")
    builder.build(sim_config=sim_config, trace=args.trace)

if __name__ == "__main__":
    main()