#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litex.gen import *

from litex.soc.interconnect.csr import *

# BitSlip ------------------------------------------------------------------------------------------

class BitSlip(Module):
    def __init__(self, dw=8, value=0, cycles=1):
        self.i = Signal(dw)
        self.o = Signal(dw)

        # # #

        self.r = r = Signal((cycles + 1)*dw, reset_less=True)
        self.sync += r.eq(Cat(r[dw:], self.i))
        cases = {}
        for i in range(cycles*dw):
            cases[dw-1-i] = self.o.eq(r[i:dw+i])
        self.comb += Case(value, cases)

# Coarse Delay -------------------------------------------------------------------------------------

class CoarseDelay(LiteXModule):
    def __init__(self, i, o, clk_domain="wr", clk_cycles=1):
        self._value = CSRStorage(3)

        # # #

        # Bitslip.
        bitslip = BitSlip(dw=8, value=self._value.storage, cycles=clk_cycles)
        bitslip = ClockDomainsRenamer(clk_domain)(bitslip)
        self.submodules += bitslip
        self.comb += bitslip.i.eq(Replicate(i, 8))

        # 8:1 Serialization.
        self.specials += Instance("OSERDESE2",
            p_DATA_WIDTH     = 8,
            p_TRISTATE_WIDTH = 1,
            p_DATA_RATE_OQ   = "DDR",
            p_DATA_RATE_TQ   = "BUF",
            p_SERDES_MODE    = "MASTER",

            i_OCE    = 1,
            i_RST    = ResetSignal(clk_domain),
            i_CLK    = ClockSignal(clk_domain + "4x"),
            i_CLKDIV = ClockSignal(clk_domain),
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
