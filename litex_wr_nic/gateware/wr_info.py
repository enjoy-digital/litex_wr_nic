#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg, BusSynchronizer

from litex.gen import *
from litex.soc.interconnect.csr import CSRStatus

# WR Management Information ------------------------------------------------------------------------

WR_INFO_MAGIC = 0x57524331 # WRC1.


class WRInfo(LiteXModule):
    """Live configuration and reset-request diagnostics, readable in sys.

    Reset reason describes resets requested through the WR host register:
    0 = FPGA/system reset, 1 = host CPU reset. It is not a firmware watchdog.
    The image hash identifies the firmware supplied at FPGA build time.
    """
    def __init__(self, host_bus, cpu_type, with_cpu_memory, memory_ready,
        link_up, time_valid, firmware_hash=0, host_size=0x0100_0000):
        self.magic         = CSRStatus(32, reset=WR_INFO_MAGIC)
        self.cpu_type      = CSRStatus(8, reset={"urv": 0, "vexriscv": 1}[cpu_type])
        self.memory_mode   = CSRStatus(1, reset=int(with_cpu_memory))
        self.memory_size   = CSRStatus(32, reset=128*1024)
        self.firmware_hash = CSRStatus(256, reset=firmware_hash)
        self.status        = CSRStatus(4)
        self.reset_reason  = CSRStatus(1)
        self.reset_count   = CSRStatus(32)

        # # #

        reset_requested = Signal()
        reset_reason_wr = Signal()
        reset_count_wr  = Signal(32)
        # Monitor completed writes to RESET in the existing WR CPU CSR bank.
        self.sync.wr_sys += If(host_bus.cyc & host_bus.stb & host_bus.ack & host_bus.we &
            host_bus.sel[0] & ((host_bus.adr & ((host_size - 1) >> 2)) == (0x20b00 >> 2)),
            reset_requested.eq(host_bus.dat_w[0]),
            If(host_bus.dat_w[0] & ~reset_requested,
                reset_reason_wr.eq(1),
                reset_count_wr.eq(reset_count_wr + 1),
            ),
        )
        self.specials += [
            MultiReg(memory_ready,    self.status.status[0]),
            MultiReg(reset_requested, self.status.status[1]),
            MultiReg(link_up,         self.status.status[2]),
            MultiReg(time_valid,      self.status.status[3]),
            MultiReg(reset_reason_wr, self.reset_reason.status),
        ]
        self.count_cdc = BusSynchronizer(32, "wr_sys", "sys")
        self.comb += [
            self.count_cdc.i.eq(reset_count_wr),
            self.reset_count.status.eq(self.count_cdc.o),
        ]
