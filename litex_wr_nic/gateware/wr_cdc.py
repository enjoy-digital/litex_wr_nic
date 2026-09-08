#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.soc.interconnect import stream

# WR Clock Crossing -------------------------------------------------------------------------------

class WRClockCrossing(LiteXModule, DUID):
    """FIFO and protocol logic share synchronized resets at both ends."""
    def __init__(self, layout, cd_from, cd_to, depth=16):
        DUID.__init__(self)
        self.cd_input        = ClockDomain(f"wr_cdc_in{self.duid}")
        self.cd_output       = ClockDomain(f"wr_cdc_out{self.duid}")
        self.cd_input_reset  = ClockDomain(f"wr_cdc_in_reset{self.duid}")
        self.cd_output_reset = ClockDomain(f"wr_cdc_out_reset{self.duid}")
        self.input_cd  = self.cd_input.name
        self.output_cd = self.cd_output.name

        # # #

        reset = ResetSignal(cd_from) | ResetSignal(cd_to)
        self.comb += [
            self.cd_input.clk.eq(ClockSignal(cd_from)),
            self.cd_output.clk.eq(ClockSignal(cd_to)),
            self.cd_input_reset.clk.eq(ClockSignal(cd_from)),
            self.cd_output_reset.clk.eq(ClockSignal(cd_to)),
        ]
        # FIFO pointers have synchronous resets. If one clock is stopped,
        # keep both sides reset until it has resumed and clocked its reset.
        # Otherwise the running side can see a stale remote pointer and replay
        # old commands before the stopped side has cleared its state.
        ready_reset = self.cd_input_reset.rst | self.cd_output_reset.rst
        self.specials += [
            AsyncResetSynchronizer(self.cd_input_reset, reset),
            AsyncResetSynchronizer(self.cd_output_reset, reset),
            AsyncResetSynchronizer(self.cd_input, ready_reset),
            AsyncResetSynchronizer(self.cd_output, ready_reset),
        ]
        if cd_from == cd_to:
            self.fifo = ClockDomainsRenamer(self.input_cd)(stream.SyncFIFO(layout, depth, buffered=True))
        else:
            self.fifo = ClockDomainsRenamer({"write": self.input_cd, "read": self.output_cd})(
                stream.AsyncFIFO(layout, depth))
        self.sink   = self.fifo.sink
        self.source = self.fifo.source
