#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import math

from migen import *
from migen.genlib.cdc       import MultiReg
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import *

from litex.soc.interconnect import stream

# BitSlip ------------------------------------------------------------------------------------------

class BitSlip(Module):
    def __init__(self, value, cycles=1):
        self.i = Signal(4)
        self.o = Signal(4)

        # # #

        self.r = r = Signal((cycles+1)*4, reset_less=True)
        self.sync += r.eq(Cat(r[4:], self.i))
        cases = {}
        for i in range(cycles*4):
            cases[4-1-i] = self.o.eq(r[i:4+i])
        self.comb += Case(value, cases)

# Coarse Delay Line --------------------------------------------------------------------------------

class CoarseDelayLine(LiteXModule):
    def __init__(self, i, o):
        self._value = CSRStorage(3)

        # # #

        # Bitslip.
        bitslip = BitSlip(value=self._value.storage, cycles=2)
        bitslip = ClockDomainsRenamer("wr")(bitslip)
        self.submodules += bitslip
        self.comb += bitslip.i.eq(Replicate(i, 4))

        # 8:1 Serialization.
        self.specials += Instance("OSERDESE2",
            p_DATA_WIDTH     = 4,
            p_TRISTATE_WIDTH = 1,
            p_DATA_RATE_OQ   = "SDR",
            p_DATA_RATE_TQ   = "BUF",
            p_SERDES_MODE    = "MASTER",

            i_OCE    = 1,
            i_RST    = ResetSignal("wr"),
            i_CLK    = ClockSignal("wr4x"),
            i_CLKDIV = ClockSignal("wr"),
            i_D1     = bitslip.o[0],
            i_D2     = bitslip.o[1],
            i_D3     = bitslip.o[2],
            i_D4     = bitslip.o[3],
            o_OQ     = o,
        )

# Fine Delay : 0-6ns.
# Coarse Delay: 0-16ns.
# White Rabbit Period: 16ns.