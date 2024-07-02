#!/usr/bin/env python3

import sys
import argparse

from migen import *

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

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

        from gateware.wrf_stream2wb import Stream2Wishbone

        self.wrf_stream2wb = Stream2Wishbone(cd_to="wr")

        wrf_snk_timer = WaitTimer(1000)
        self.submodules += wrf_snk_timer
        self.comb += wrf_snk_timer.wait.eq(~wrf_snk_timer.done)

        self.wrf_conv = stream.Converter(16, 8)

        self.specials += Instance("wrf_snk_test",
            i_wrf_clk   = ClockSignal("sys"),
            i_wrf_send  = wrf_snk_timer.done,
            o_wrf_valid = self.wrf_conv.sink.valid,
            i_wrf_ready = self.wrf_conv.sink.ready,
            o_wrf_data  = self.wrf_conv.sink.data,
        )
        self.platform.add_source("gateware/wrf_snk_test.v")

        self.comb += self.wrf_conv.source.connect(self.wrf_stream2wb.sink)

        self.comb += self.wrf_stream2wb.bus.ack.eq(1)

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