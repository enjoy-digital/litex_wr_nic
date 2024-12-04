#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

# PPS Generator ------------------------------------------------------------------------------------

class PPSGenerator(LiteXModule):
    def __init__(self, i, o,  clk_domain, clk_freq, duty_cycle=20/100):
        timer = WaitTimer(clk_freq*duty_cycle)
        timer = ClockDomainsRenamer(clk_domain)(timer)
        self.add_module(name="timer", module=timer)
        self.comb += [
            timer.wait.eq(~i),
            o.eq(~timer.done),
        ]
