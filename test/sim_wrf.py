#!/usr/bin/env python3

import sys
import argparse

from migen import *
from migen.genlib.misc import timeline

from litex.tools.litex_sim import *

sys.path.append("..")

from gateware.wrf_wb2stream import Wishbone2Stream
from gateware.wrf_stream2wb import Stream2Wishbone

# Packer Streamer ----------------------------------------------------------------------------------

class PacketStreamer(Module):
    def __init__(self, data_width, datas):
        self.source = source = stream.Endpoint([("data", data_width)])

        # # #

        count = Signal(max=len(datas))

        mem  = Memory(data_width, len(datas), init=datas)
        port = mem.get_port(async_read=True)
        self.specials += mem, port

        self.comb += [
            port.adr.eq(count),
            source.valid.eq(1),
            source.first.eq(count == 0),
            source.last.eq( count == (len(datas) - 1)),
            source.data.eq(port.dat_r),
        ]
        self.sync += [
            If(source.valid & source.ready,
                If(source.last,
                    count.eq(0)
                ).Else(
                    count.eq(count + 1)
                )
            )
        ]

# Packet Checker -----------------------------------------------------------------------------------

class PacketChecker(Module):
    def __init__(self, data_width, datas, with_framing_error=True):
        self.data_width    = data_width
        self.sink          = sink = stream.Endpoint([("data", data_width)])
        self.data_error    = Signal()
        self.framing_error = Signal()
        self.reference     = Signal(data_width)
        self.loop          = Signal(16)

        # # #

        count = Signal(max=len(datas))

        mem = Memory(data_width, len(datas), init=datas)
        port = mem.get_port(async_read=True)
        self.specials += mem, port

        # Data/Framing Check.
        self.comb += [
            port.adr.eq(count),
            sink.ready.eq(1),
            self.reference.eq(port.dat_r),
            If(sink.valid & sink.ready,
                # Data Check.
                If(sink.data != self.reference,
                    self.data_error.eq(1)
                ),
                # Framing Check.
                If(count == (len(datas) - 1),
                    If(sink.last == 0,
                        self.framing_error.eq(with_framing_error)
                    )
                )
            )
        ]

        # Loop/Count Increment.
        self.sync += [
            If(sink.valid & sink.ready,
                If(count == (len(datas) - 1),
                    count.eq(0),
                    self.loop.eq(self.loop + 1)
                ).Else(
                    count.eq(count + 1)
                )
            )
        ]

    def add_debug(self, banner):
        last_loop = Signal(32)
        data_error_msg = " Data Error: 0x\%0{}x vs 0x\%0{}x".format(
            self.data_width//4,
            self.data_width//4)
        framing_error_msg = " Framing Error"
        self.sync += [
            If(self.data_error,
                Display(banner + data_error_msg,
                    self.sink.data,
                    self.reference
                )
            ),
            If(self.framing_error,
                Display(banner + framing_error_msg)
            ),
            If(last_loop != self.loop,
                Display(banner + " Loop: %d", self.loop),
                last_loop.eq(self.loop)
            ),
            timeline(self.data_error, [
                (128, [Finish()])
            ])
        ]

# WRCSim -------------------------------------------------------------------------------------------

class WRCSim(SimSoC):
    def __init__(self, length=16):
        SimSoC.__init__(self, cpu_type="None", with_uart=False)

        self.cd_wr = ClockDomain()
        self.sync += self.cd_wr.clk.eq(~self.cd_wr.clk)

        data = [i%256 for i in range(length)]

        # Streamer -> Stream2Wishbone -> Whishbone2Stream -> Checker -------------------------------

        self.streamer  = PacketStreamer(8, data)
        self.stream2wb = Stream2Wishbone(cd_to="wr")
        self.wb2stream = Wishbone2Stream(cd_from="wr")
        self.checker   = PacketChecker(8, data, with_framing_error=True)
        self.checker.add_debug("[Checker]")
        self.comb += [
            self.streamer.source.connect(self.stream2wb.sink),
            self.stream2wb.bus.connect(self.wb2stream.bus),
            self.wb2stream.source.connect(self.checker.sink),
        ]

        # Sim Finish -------------------------------------------------------------------------------
        cycles = Signal(32)
        self.sync += cycles.eq(cycles + 1)
        self.sync += If(cycles == 1000, Finish())

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="White Rabbit WRF Interface Simulation")
    parser.add_argument("--length",      default=16, type=int,help="Packet Data Length.")
    parser.add_argument("--trace",       action="store_true", help="Enable Tracing")
    parser.add_argument("--trace-fst",   action="store_true", help="Enable FST tracing (default=VCD)")
    parser.add_argument("--trace-start", default=0,           help="Cycle to start tracing")
    parser.add_argument("--trace-end",   default=-1,          help="Cycle to end tracing")
    args = parser.parse_args()

    sim_config = SimConfig(default_clk="sys_clk")

    # SoC ------------------------------------------------------------------------------------------
    soc = WRCSim(length=args.length)

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
