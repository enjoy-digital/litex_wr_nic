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
        self.i     = Signal(8)
        self.o     = Signal(8)

        # # #
        self.r = r = Signal((cycles+1)*8, reset_less=True)
        self.sync += r.eq(Cat(r[8:], self.i))
        cases = {}
        for i in range(cycles*8):
            cases[i] = self.o.eq(r[i:8+i])
        self.comb += Case(value, cases)

# Coarse Delay Line --------------------------------------------------------------------------------

class CoarseDelayLine(LiteXModule):
    def __init__(self, i, o):
        self._value = CSRStorage(3)

        # # #

        # Bitslip.
        self.bitslip = bitslip = BitSlip(value=self._value.storage, cycles=1)
        self.comb += bitslip.i.eq(Replicate(i, 8))

        # 8:1 Serialization.
        self.specials += Instance("OSERDESE2",
            p_DATA_WIDTH     = 8,
            p_TRISTATE_WIDTH = 1,
            p_DATA_RATE_OQ   = "SDR",
            p_DATA_RATE_TQ   = "BUF",
            p_SERDES_MODE    = "MASTER",

            i_OCE    = 1,
            i_RST    = ResetSignal("wr"),
            i_CLK    = ClockSignal("wr8x"),
            i_CLKDIV = ClockSignal("wr"),
            i_D1     = bitslip.o[0],
            i_D2     = bitslip.o[1],
            i_D3     = bitslip.o[2],
            i_D4     = bitslip.o[3],
            i_D5     = bitslip.o[4],
            i_D6     = bitslip.o[5],
            i_D7     = bitslip.o[6],
            i_D8     = bitslip.o[7],
            o_OQ     = o,
        )
