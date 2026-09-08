#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os

from migen import *
from migen.genlib.cdc import BusSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import *

from litex_wr_nic.gateware.wr_clock import WRTuningCDC

# AD5683R DAC --------------------------------------------------------------------------------------

class AD5683RDAC(LiteXModule):
    def __init__(self, platform, pads, load, value, gain=1, clk_domain="wr"):
        assert gain in [1, 2]
        self._force   = CSRStorage()
        self._load    = CSRStorage(1)
        self._value   = CSRStorage(16)
        self._current = CSRStatus(16)

        # # #

        # Transfer each host command as one coherent value/mode/load record.
        # The WR tuning interface is already synchronous to clk_domain.
        self.host_cdc = host_cdc = WRTuningCDC(18, "sys", clk_domain)
        driver_cd = host_cdc.output_cd
        forced       = Signal()
        manual_value = Signal(16)
        manual_load  = Signal()
        load_i       = Signal()
        value_i      = Signal(16)
        self.comb += [
            host_cdc.sink.load.eq(self._force.re | (self._load.re & self._load.storage)),
            host_cdc.sink.data.eq(Cat(self._value.storage, self._force.storage,
                self._load.re & self._load.storage)),
            load_i.eq(Mux(forced, manual_load, load)),
            value_i.eq(Mux(forced, manual_value, value)),
        ]
        sync = getattr(self.sync, driver_cd)
        sync += [
            manual_load.eq(0),
            If(host_cdc.source.load,
                forced.eq(host_cdc.source.data[16]),
                If(host_cdc.source.data[17],
                    manual_value.eq(host_cdc.source.data[:16]),
                    manual_load.eq(host_cdc.source.data[16]),
                ),
            ),
        ]
        self.current_cdc = BusSynchronizer(16, driver_cd, "sys")
        self.comb += [self.current_cdc.i.eq(value_i), self._current.status.eq(self.current_cdc.o)]

        # DAC Driver Instance.
        self.specials += Instance("serial_dac_arb",
            p_g_invert_sclk    = 0,
            p_g_num_data_bits  = 16,
            p_g_num_extra_bits = 8,
            p_g_enable_x2_gain = {1: 0, 2: 1}[gain],

            i_clk_i        = ClockSignal(driver_cd),
            i_rst_n_i      = ~ResetSignal(driver_cd),

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
        cdir = os.path.abspath(os.path.dirname(__file__))
        platform.add_source(os.path.join(cdir, "serial_dac.vhd"))
        platform.add_source(os.path.join(cdir, "serial_dac_arb.vhd"))
