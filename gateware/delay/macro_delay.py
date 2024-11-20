#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litex.gen import *

from litex.soc.interconnect.csr import *

# Macro Delay --------------------------------------------------------------------------------------

class MacroDelay(LiteXModule):
    def __init__(self, i, o, clk_domain="sys", default_value=1):
        self._value = CSRStorage(32, description="Macro Delay Clk Cycles.", reset=default_value)

        # # #

        # Sync.
        _sync = getattr(self.sync, clk_domain)

        # Input Rising Edge Detection.
        i_d      = Signal()
        i_rising = Signal()
        _sync += i_d.eq(i)
        self.comb += i_rising.eq(i & ~i_d)

        # Delay.
        enable = Signal()
        count  = Signal(32)
        _sync += [
            If(i_rising,
                enable.eq(1),
                count.eq(self._value.storage - 1),
            ),
            If(count > 0,
                count.eq(count - 1)
            ).Else(
                enable.eq(0),
            )
        ]

        # Output.
        self.comb += o.eq(enable & (count == 0))
