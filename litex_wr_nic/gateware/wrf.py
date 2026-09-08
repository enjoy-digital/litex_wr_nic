#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import BusSynchronizer
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *
from litex.soc.interconnect import stream, wishbone
from litex.soc.interconnect.csr import CSRStatus

# WR Fabric Interfaces -----------------------------------------------------------------------------

class WRFInterface(wishbone.Interface):
    def __init__(self):
        super().__init__(data_width=16, address_width=2, addressing="byte")
        # Unlike classic Wishbone, WR fabric separates request acceptance
        # (STB and not STALL) from response completion (ACK/ERR/RTY).
        for name in ("stall", "rty"):
            setattr(self, name, Signal(name=name))
            self.layout.append((name, 1, DIR_S_TO_M))


class WRFClockCrossing(LiteXModule, DUID):
    """FIFO and protocol logic share synchronized resets at both ends."""
    def __init__(self, layout, cd_from, cd_to, depth=16):
        DUID.__init__(self)
        self.cd_input  = ClockDomain(f"wrf_in{self.duid}")
        self.cd_output = ClockDomain(f"wrf_out{self.duid}")
        self.input_cd  = self.cd_input.name
        self.output_cd = self.cd_output.name
        reset = ResetSignal(cd_from) | ResetSignal(cd_to)
        self.comb += [
            self.cd_input.clk.eq(ClockSignal(cd_from)),
            self.cd_output.clk.eq(ClockSignal(cd_to)),
        ]
        self.specials += [
            AsyncResetSynchronizer(self.cd_input, reset),
            AsyncResetSynchronizer(self.cd_output, reset),
        ]
        if cd_from == cd_to:
            self.fifo = ClockDomainsRenamer(self.input_cd)(stream.SyncFIFO(layout, depth, buffered=True))
        else:
            self.fifo = ClockDomainsRenamer({"write": self.input_cd, "read": self.output_cd})(
                stream.AsyncFIFO(layout, depth))
        self.sink   = self.fifo.sink
        self.source = self.fifo.source


class WRFCounters(LiteXModule):
    def __init__(self, cd):
        for name in ("packets", "errors", "stalls", "overflow"):
            counter = Signal(32, name=name)
            status = CSRStatus(32, name=name)
            cdc = BusSynchronizer(32, cd, "sys")
            setattr(self, name, counter)
            setattr(self, "_" + name, status)
            setattr(self, name + "_cdc", cdc)
            self.comb += [cdc.i.eq(counter), status.status.eq(cdc.o)]
