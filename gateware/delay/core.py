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
    def __init__(self, dw, i=None, o=None, cycles=1):
        self.i   = Signal(dw) if i is None else i
        self.o   = Signal(dw) if o is None else o
        self.rst = Signal() if rst is None else rst
        self.inc = Signal() if inc is None else inc

        # # #

        self.value = value = Signal(max=(cycles+1)*dw)
        self.sync += [
            If(self.inc,
                value.eq(value + 1),
                If(value == (dw - 1),
                    value.eq(0)
                )
            )
        ]
        self.sync += If(self.rst, value.eq(0))

        self.r = r = Signal((cycles+1)*dw, reset_less=True)
        self.sync += r.eq(Cat(r[dw:], self.i))
        cases = {}
        for i in range(cycles*dw):
            cases[i] = self.o.eq(r[i:dw+i])
        self.comb += Case(value, cases)

# Coarse Delay Line --------------------------------------------------------------------------------

class CoarseDelayLine(LiteXModule):
    def __init__(self, platform, i, o):
        self._value = CSRStorage(3)

        # # #

        # Signals.
        bitslip_i = Signal(8)
        bitslip_o = Signal(8)

        self.comb += bitslip_i.eq(Replicate(i, 8))

        # Bitslip.
        self.bitslip = BitSlip(dw=8, rst=0, i=bitslip_i, o=bitslip_o, cycles=1)

        # 8:1 Serialization.
        self.specials += Instance("OSERDESE2",
            p_DATA_WIDTH     = 8,
            p_TRISTATE_WIDTH = 1,
            p_DATA_RATE_OQ   = "DDR",
            p_DATA_RATE_TQ   = "BUF",
            p_SERDES_MODE    = "MASTER",

            i_OCE    = 1,
            i_RST    = ResetSignal("wr"),
            i_CLK    = ClockSignal("wr4x"),
            i_CLKDIV = ClockSignal("wr"),
            i_D1     = bitslip_o[0],
            i_D2     = bitslip_o[1],
            i_D3     = bitslip_o[2],
            i_D4     = bitslip_o[3],
            i_D5     = bitslip_o[4],
            i_D6     = bitslip_o[5],
            i_D7     = bitslip_o[6],
            i_D8     = bitslip_o[7],
            o_OQ     = o,
        )
