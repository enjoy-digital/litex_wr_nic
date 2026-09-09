#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg, BusSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import CSRField, CSRStatus

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
        self.magic = CSRStatus(32, reset=WR_INFO_MAGIC,
            description="WR management signature (WRC1).")
        self.cpu_type = CSRStatus(8, reset={"urv": 0, "vexriscv": 1}[cpu_type],
            description="WR CPU implementation: 0 = uRV, 1 = VexRiscv.")
        self.memory_mode = CSRStatus(1, reset=int(with_cpu_memory),
            description="WR CPU memory: 0 = private RAM, 1 = SoC memory.")
        self.memory_size = CSRStatus(32, reset=128*1024,
            description="Reserved WR CPU memory size in bytes.")
        self.firmware_hash = CSRStatus(256, reset=firmware_hash,
            description="SHA-256 of the firmware supplied at FPGA build time.")
        self.status = CSRStatus(fields=[
            CSRField("memory_ready",    size=1, description="WR CPU memory is ready."),
            CSRField("reset_requested", size=1, description="Host CPU reset is asserted."),
            CSRField("link_up",         size=1, description="WR link is up."),
            CSRField("time_valid",      size=1, description="WR core marks its time valid."),
        ])
        self.reset_reason = CSRStatus(1,
            description="Last reset request: 0 = system reset, 1 = host CPU reset.")
        self.reset_count = CSRStatus(32,
            description="Host CPU reset requests, wrapping at 32 bits.")

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
            MultiReg(memory_ready,    self.status.fields.memory_ready),
            MultiReg(reset_requested, self.status.fields.reset_requested),
            MultiReg(link_up,         self.status.fields.link_up),
            MultiReg(time_valid,      self.status.fields.time_valid),
            MultiReg(reset_reason_wr, self.reset_reason.status),
        ]
        self.count_cdc = BusSynchronizer(32, "wr_sys", "sys")
        self.comb += [
            self.count_cdc.i.eq(reset_count_wr),
            self.reset_count.status.eq(self.count_cdc.o),
        ]
