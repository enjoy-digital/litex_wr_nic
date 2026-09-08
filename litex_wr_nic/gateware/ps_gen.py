#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litex_wr_nic.gateware.wr_clock import WRMMCMBackend

# Phase Shift Generator ---------------------------------------------------------------------------

class PSGen(WRMMCMBackend):
    """Compatibility constructor; integrations must connect psdone."""
    def __init__(self, cd_psclk, cd_sys, ctrl_size=16, div_n=0, **kwargs):
        super().__init__(cd_psclk, cd_sys, ctrl_size, div_n, **kwargs)
        self.ctrl_data = self.command.data
        self.ctrl_load = self.command.load
