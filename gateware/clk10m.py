#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

# Clk 10MHz Generator ------------------------------------------------------------------------------

class Clk10MGenerator(LiteXModule):
    def __init__(self, pulse_i, clk10m_o, clk_domain="wr8x"):

        # Sync.
        _sync = getattr(self.sync, clk_domain)

        # 10MHz Gen from 500MHz.
        count = Signal(8)
        _sync += [
            If(pulse_i,
                count.eq(0),
                clk10m_o.eq(0),
            ).Else(
                count.eq(count + 1),
                If(count == (25 - 1),
                    count.eq(0),
                    clk10m_o.eq(~clk10m_o)
                )
            )
        ]

