#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# NB6L295 Delay Line -------------------------------------------------------------------------------

# Dual-channel programmable delay chip with 3.2 ns fixed minimum delay per channel and 0-6 ns
# programmable range in ~11 ps steps. Operates in Dual Delay Mode, controlled via a 3-pin Serial
# Data Interface.

class NB6L295DelayLine(LiteXModule):
    def __init__(self, platform, pads):
        self._req   = CSRStorage()
        self._sel   = CSRStorage()
        self._value = CSRStorage(9)
        self._busy  = CSRStatus()

        # # #

        # Delay Line Driver Instance.
        self.specials += Instance("fine_delay_ctrl",
            i_rst_sys_n_i       = ~ResetSignal("sys"),
            i_clk_sys_i         = ClockSignal("sys"),

            i_fine_dly_req_i    = self._req.re,
            i_fine_dly_sel_i    = self._sel.storage,
            i_fine_dly_values_i = self._value.storage,
            o_fine_dly_busy_o   = self._busy.status,

            o_delay_en_o        = pads.en,
            o_delay_sclk_o      = pads.sclk,
            o_delay_sload_o     = pads.sload,
            o_delay_sdin_o      = pads.sdin,
        )

        self.add_sources(platform)

    def add_sources(self, platform):
       platform.add_source("gateware/nb6l295/fine_delay_ctrl.vhd")
