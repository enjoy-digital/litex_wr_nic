#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer

from litex.gen import *
from litex.soc.interconnect.csr import *

# Time Generator -----------------------------------------------------------------------------------

class TimeGenerator(LiteXModule):
    def __init__(self, clk_domain, clk_freq, with_csr=True):
        assert 1e9/clk_freq == int(1e9/clk_freq)

        self.enable      = Signal()
        self.sync_enable = Signal()
        self.write       = Signal()
        self.write_time  = Signal(64)
        self.time_inc    = Signal(32, reset=int(1e9/clk_freq) << 24)

        # Time Sync Interface.
        self.time_sync    = Signal()
        self.time_seconds = Signal(40)

        # # #

        # Time Signals.
        self.time = time = Signal(64)
        self.frac = frac = Signal(24)

        # Time Clk Domain.
        self.cd_time = ClockDomain()
        self.comb += [
            self.cd_time.clk.eq(ClockSignal(clk_domain)),
            self.cd_time.rst.eq(ResetSignal(clk_domain)),
        ]

        # Time Handling.
        self.sync.time += [
            # Disable: Reset Time to 0.
            If(~self.enable,
                time.eq(0),
                frac.eq(0),
            # Software Write.
            ).Elif(self.write,
                time.eq(self.write_time),
                frac.eq(0),
            # Time Sync.
            ).Elif(self.sync_enable & self.time_sync,
                time.eq(((self.time_seconds + 1)* int(1e9))),
                frac.eq(0),
            # Increment.
            ).Else(
                Cat(frac, time).eq(Cat(frac, time) + self.time_inc),
            )
        ]

        # CSRs.
        if with_csr:
            self.add_csr(clk_domain)

    def add_csr(self, clk_domain, default_enable=1, default_pps_align=1):
        self._control = CSRStorage(fields=[
            CSRField("enable", size=1, offset=0, values=[
                ("``0b0``", "Time Generator Disabled."),
                ("``0b1``", "Time Generator Enabled."),
            ], reset=default_enable),
            CSRField("read",  size=1, offset=1, pulse=True),
            CSRField("write", size=1, offset=2, pulse=True),
            CSRField("sync_enable", size=1, offset=3, values=[
                ("``0b0``", "Sync (Re-)Alignment Disabled."),
                ("``0b1``", "Sync (Re-)Alignment Enabled."),
            ], reset=default_pps_align),
        ])
        self._read_time  = CSRStatus(64,  description="Read Time  (ns) (FPGA Time -> SW).")
        self._write_time = CSRStorage(64, description="Write Time (ns) (SW Time -> FPGA).")
        self._time_inc   = CSRStorage(32, reset=self.time_inc.reset, description="Time Increment in ns per tick (Q8.24 format).")

        # # #

        # Enable.
        self.specials += MultiReg(self._control.fields.enable, self.enable)

        # Sync.
        self.specials += MultiReg(self._control.fields.sync_enable, self.sync_enable)

        # Time Read (FPGA -> SW).
        time_read = Signal(64)
        time_read_ps = PulseSynchronizer("sys", "time")
        self.submodules += time_read_ps
        self.comb += time_read_ps.i.eq(self._control.fields.read)
        self.sync.time += If(time_read_ps.o, time_read.eq(self.time))
        self.specials += MultiReg(time_read, self._read_time.status)

        # Time Write (SW -> FPGA).
        self.specials += MultiReg(self._write_time.storage, self.write_time, "time")
        time_write_ps = PulseSynchronizer("sys", "time")
        self.submodules += time_write_ps
        self.comb += time_write_ps.i.eq(self._control.fields.write)
        self.comb += self.write.eq(time_write_ps.o)

        # Time Increment.
        self.specials += MultiReg(self._time_inc.storage, self.time_inc, "time")
