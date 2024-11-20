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
    def __init__(self, pulse_i, pulse_o, clk_domain="sys", default_delay=1):
        self._value = CSRStorage(32, description="Macro Delay Clk Cycles.", reset=default_delay)

        # # #

        # Sync.
        _sync = getattr(self.sync, clk_domain)

        # Delay.
        self.enable = enable = Signal()
        self.count  = count  = Signal(32)
        _sync += [
            If(pulse_i,
                enable.eq(1),
                count.eq(self._value.storage - 1),
            ).Else(
                If(count == 0,
                    enable.eq(0)
                ).Else(
                    count.eq(count - 1)
                )
            )
        ]

        # Output.
        self.comb += pulse_o.eq(enable & (count == 0))
