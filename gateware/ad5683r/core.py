#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# AD5683R DAC --------------------------------------------------------------------------------------

class AD5683RDAC(LiteXModule):
    def __init__(self, platform, pads, load, value, gain=1):
        assert gain in [1, 2]
        self._force = CSRStorage()
        self._load  = CSRStorage(1)
        self._value = CSRStorage(16)

        # # #

        # Signals.
        load_i  = Signal()
        value_i = Signal(16)

        # Force/Load.
        self.sync.wr += [
            If(self._force.storage,
                load_i.eq(self._load.storage),
                value_i.eq(self._value.storage),
            ).Else(
                load_i.eq(load),
                value_i.eq(value),
            )
        ]

        # DAC Driver Instance.
        self.specials += Instance("serial_dac_arb",
            p_g_invert_sclk    = 0,
            p_g_num_data_bits  = 16,
            p_g_num_extra_bits = 8,
            p_g_x2_gain        = {1: 0, 2: 1}[gain],

            i_clk_i        = ClockSignal("wr"),
            i_rst_n_i      = ~ResetSignal("wr"),

            i_val_i        = value_i,
            i_load_i       = load_i,

            o_dac_ldac_n_o = pads.ldac_n,
            o_dac_clr_n_o  = Open(),
            o_dac_sync_n_o = pads.sync_n,
            o_dac_sclk_o   = pads.sclk,
            o_dac_din_o    = pads.sdi,
        )
        self.add_sources(platform)

    def add_sources(self, platform):
       platform.add_source("gateware/ad5683r/serial_dac.vhd")
       platform.add_source("gateware/ad5683r/serial_dac_arb.vhd")
