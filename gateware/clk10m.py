#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litex.gen import *

from litex.soc.interconnect import stream

# Clk 10MHz Generator ------------------------------------------------------------------------------

class Clk10MGenerator(LiteXModule):
    def __init__(self, pulse_i, clk10m_o, clk_domain="wr"):
        # Gearbox for 50-bit input to 8-bit output.
        gearbox = stream.Gearbox(i_dw=50, o_dw=8, msb_first=False)
        gearbox = ResetInserter()(gearbox)
        gearbox = ClockDomainsRenamer(clk_domain)(gearbox)
        self.add_module(name="gearbox", module=gearbox)

        # Gearbox Synchronization.
        self.comb += gearbox.reset.eq(pulse_i)

        # Gearbox Input.
        self.comb += [
            gearbox.sink.valid.eq(1),
            gearbox.sink.data[:25].eq(0b0000000000000000000000000),
            gearbox.sink.data[25:].eq(0b1111111111111111111111111),
        ]

        # Gearbox Output.
        self.comb += [
            gearbox.source.ready.eq(1),
            clk10m_o.eq(gearbox.source.data)
        ]
