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

# NB6L295 Delay Line -------------------------------------------------------------------------------

# Dual-channel programmable delay chip with 3.2 ns fixed minimum delay per channel and 0-6 ns
# programmable range in ~11 ps steps. Operates in Dual Delay Mode, controlled via a 3-pin Serial
# Data Interface.

class NB6L295DelayLine(LiteXModule):
    def __init__(self, platform, pads, clk_divider=16):
        self._req   = CSRStorage()
        self._sel   = CSRStorage()
        self._value = CSRStorage(9)
        self._busy  = CSRStatus()

        # # #

        # Signal.
        busy = Signal()

        # Create slow Delay Clk.
        self.cd_delay = ClockDomain()
        delay_clk_counter = Signal(int(math.log2(clk_divider)))
        self.sync += delay_clk_counter.eq(delay_clk_counter + 1)
        self.sync += self.cd_delay.clk.eq(delay_clk_counter[-1])
        self.specials += AsyncResetSynchronizer(self.cd_delay, ResetSignal("sys")),

        # CDC.
        self.cdc = cdc = stream.ClockDomainCrossing(
            layout  = [("sel", 1), ("value", 9)],
            cd_from = "sys",
            cd_to   = "delay",
        )
        self.comb += [
            cdc.sink.valid.eq(self._req.re),
            cdc.sink.sel.eq(self._sel.storage),
            cdc.sink.value.eq(self._value.storage),
            cdc.source.ready.eq(1), # Always ready since busy is checked before req.
        ]

        # Delay Line Driver Instance.
        self.specials += Instance("fine_delay_ctrl",
            i_rst_sys_n_i       = ~ResetSignal("delay"),
            i_clk_sys_i         = ClockSignal("delay"),

            i_fine_dly_req_i    = cdc.source.valid,
            i_fine_dly_sel_i    = cdc.source.sel,
            i_fine_dly_values_i = cdc.source.value,
            o_fine_dly_busy_o   = busy,

            o_delay_en_o        = pads.en,
            o_delay_sclk_o      = pads.sclk,
            o_delay_sload_o     = pads.sload,
            o_delay_sdin_o      = pads.sdin,
        )
        self.specials += MultiReg(busy, self._busy.status)

        self.add_sources(platform)

    def add_sources(self, platform):
       platform.add_source("gateware/nb6l295/fine_delay_ctrl.vhd")
