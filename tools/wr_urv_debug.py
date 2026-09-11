#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
"""Bounded uRV debug memory access through the WR host CPU CSR window.

Uses the instruction/mailbox protocol implemented by WRPC tools/wrpc.c.
The CPU is briefly halted, a0/a1 are saved and restored, then EBREAK resumes
execution. Only use outside an uninterrupted timing observation: interrupts
are disabled during debug access, so the software PLL is paused too.
"""

import time


class URVDebug:
    def __init__(self, bus):
        self.bus = bus
        self.base = bus.mems.wr_wb_slave.base + 0xB00
        self.saved = {}
        self.entered = False

    def wait(self, offset, expected):
        deadline = time.monotonic() + 2
        while self.bus.read(self.base + offset) & 1 != expected:
            if time.monotonic() > deadline:
                raise TimeoutError("uRV debug handshake timed out")
            time.sleep(0.001)

    def instruction(self, value):
        self.wait(0x88, 1)
        self.bus.write(self.base + 0x8C, value)
        # Allow the CPU pipeline to retire the instruction before consulting
        # READY. The upstream debugger uses three explicit NOP instructions.
        time.sleep(0.00001)
        self.wait(0x88, 1)

    def read_register(self, reg):
        self.instruction(0x7D001073 | (reg << 15))
        return self.bus.read(self.base + 0x90)

    def write_register(self, reg, value):
        self.bus.write(self.base + 0x90, value)
        self.instruction(0x7D002073 | (reg << 7))

    def __enter__(self):
        if self.bus.read(self.base) & 1 or self.bus.read(self.base + 0x80) & 1:
            raise RuntimeError(
                "uRV is already reset or halted; refusing to alter its debug session"
            )
        self.bus.write(self.base + 0x84, 1)
        try:
            self.wait(0x80, 1)
            self.entered = True
            self.bus.write(self.base + 0x84, 0)
            for reg in (10, 11):
                self.saved[reg] = self.read_register(reg)
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *error):
        self.bus.write(self.base + 0x84, 0)
        if self.entered:
            # Resume only after restoring every modified register. A transport
            # failure here leaves the CPU halted and requires recovery/reload.
            for reg, value in self.saved.items():
                self.write_register(reg, value)
            # EBREAK toggles debug mode; unlike a CPU reset, it preserves
            # the firmware's execution context and private RAM.
            self.wait(0x88, 1)
            self.bus.write(self.base + 0x8C, 0x00100073)
            self.wait(0x80, 0)
            self.entered = False

    def read(self, address):
        if not self.entered or address & 3:
            raise ValueError("Aligned word access requires an active uRV debug context")
        self.write_register(10, address)
        self.instruction(0x00052583)  # lw a1, 0(a0)
        return self.read_register(11)

    def write(self, address, value):
        if not self.entered or address & 3:
            raise ValueError("Aligned word access requires an active uRV debug context")
        self.write_register(10, address)
        self.write_register(11, value)
        self.instruction(0x00B52023)  # sw a1, 0(a0)
