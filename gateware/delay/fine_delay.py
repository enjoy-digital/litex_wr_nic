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

# Fine Delay ---------------------------------------------------------------------------------------

# Dual-channel programmable delay chip with 3.2 ns fixed minimum delay per channel and 0-6 ns
# programmable range in ~11 ps steps. Operates in Dual Delay Mode, controlled via a 3-pin Serial
# Data Interface.

class FineDelay(LiteXModule):
    def __init__(self, pads, clk_divider=16):
        self._channel = CSRStorage()
        self._value   = CSRStorage(9)

        # # #

        # Create Fine Delay Clk.
        self.cd_fine_delay = ClockDomain()
        fine_delay_count = Signal(int(math.log2(clk_divider)))
        self.sync += fine_delay_count.eq(fine_delay_count + 1)
        self.sync += self.cd_fine_delay.clk.eq(fine_delay_count[-1])
        self.specials += AsyncResetSynchronizer(self.cd_fine_delay, ResetSignal("sys")),

        # CDC.
        self.cdc = cdc = stream.ClockDomainCrossing(
            layout  = [("channel", 1), ("value", 9)],
            cd_from = "sys",
            cd_to   = "fine_delay",
        )
        self.comb += [
            cdc.sink.valid.eq(self._value.re),
            cdc.sink.channel.eq(self._channel.storage),
            cdc.sink.value.eq(self._value.storage),
            cdc.source.ready.eq(1), # Always ready since busy is checked before req.
        ]

        # Delay Line Driver Instance.
        self.specials += Instance("fine_delay_ctrl",
            i_rst_sys_n_i       = ~ResetSignal("fine_delay"),
            i_clk_sys_i         = ClockSignal("fine_delay"),

            i_fine_dly_req_i    = cdc.source.valid,
            i_fine_dly_sel_i    = cdc.source.channel,
            i_fine_dly_values_i = cdc.source.value,
            o_fine_dly_busy_o   = Open(), # Unused since update are slow.

            o_delay_en_o        = pads.en,
            o_delay_sclk_o      = pads.sclk,
            o_delay_sload_o     = pads.sload,
            o_delay_sdin_o      = pads.sdin,
        )

        # Sources.
        self.add_sources()

    def add_sources(self):
        platform = LiteXContext.platform
        platform.add_source("gateware/delay/fine_delay_ctrl.vhd")
