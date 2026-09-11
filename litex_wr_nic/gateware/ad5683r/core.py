#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os

from migen import *
from migen.genlib.cdc import BusSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import *

from litex_wr_nic.gateware.wr_cdc import WRClockCrossing

# AD5683R DAC --------------------------------------------------------------------------------------

class AD5683RDAC(LiteXModule):
    def __init__(self, platform, pads, load, value, gain=2, clk_domain="wr"):
        # Preserve the old effective default; gain=1 explicitly selects the x1 range.
        assert gain in [1, 2]
        self._force   = CSRStorage(description="Select host control; takes effect in command order.")
        self._load    = CSRStorage(1, description="Write 1 to queue one host load; writing 0 has no effect.")
        self._value   = CSRStorage(16, description="Host code sampled when load is written.")
        self._current = CSRStatus(16, description="Last command accepted by the DAC driver, synchronized to sys; not analog readback.")
        self._status  = CSRStatus(fields=[
            CSRField("ready", description="A force/load write can be queued."),
            CSRField("overflow", description="A force/load write was dropped while not ready; cleared by reset."),
            CSRField("forced", description="Host control has taken effect in the DAC clock domain."),
        ])

        # # #

        # Transfer mode changes and load snapshots together. Synchronizing a
        # pulse independently of the value would allow later writes to tear a
        # command or change its ownership before it reaches the DAC.
        self.commands = commands = WRClockCrossing(
            [("force", 1), ("load", 1), ("value", 16)], "sys", clk_domain, depth=4)
        host_load = self._load.re & self._load.storage
        ready     = commands.sink.ready & ~ResetSignal(commands.input_cd)
        overflow  = Signal()
        self.comb += [
            commands.sink.valid.eq(self._force.re | host_load),
            commands.sink.force.eq(self._force.storage),
            commands.sink.load.eq(host_load),
            commands.sink.value.eq(self._value.storage),
            self._status.fields.ready.eq(ready),
            self._status.fields.overflow.eq(overflow),
        ]
        input_sync = getattr(self.sync, commands.input_cd)
        input_sync += If(commands.sink.valid & ~ready, overflow.eq(1))

        load_i    = Signal()
        value_i   = Signal(16)
        current   = Signal(16)
        forced    = Signal()
        accepted  = commands.source.valid & commands.source.ready
        next_force = Mux(accepted, commands.source.force, forced)
        # Leave a low cycle after a load before accepting another host command.
        self.comb += commands.source.ready.eq(~load_i)
        sync = getattr(self.sync, commands.output_cd)
        sync += [
            load_i.eq(0),
            If(accepted, forced.eq(commands.source.force)),
            If(accepted & commands.source.force & commands.source.load,
                load_i.eq(1),
                value_i.eq(commands.source.value),
            ).Elif(~next_force & load,
                load_i.eq(1),
                value_i.eq(value),
            ),
            # The SPI arbiter samples these signals on the same edge. It can
            # coalesce pending updates; current reports the accepted command,
            # not completion of the serial transfer or the output voltage.
            If(load_i, current.eq(value_i)),
        ]
        self.readback = BusSynchronizer(17, commands.output_cd, "sys")
        self.comb += [
            self.readback.i.eq(Cat(current, forced)),
            self._current.status.eq(self.readback.o[:16]),
            self._status.fields.forced.eq(self.readback.o[16]),
        ]

        # DAC Driver Instance.
        self.specials += Instance("serial_dac_arb",
            p_g_invert_sclk    = 0,
            p_g_num_data_bits  = 16,
            p_g_num_extra_bits = 8,
            p_g_enable_x2_gain = {1: 0, 2: 1}[gain],

            i_clk_i        = ClockSignal(clk_domain),
            i_rst_n_i      = ~ResetSignal(commands.output_cd),

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
