#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Exercise WR memory masters through the real SoC remapper and decoder."""

import pytest

from migen import run_simulation

from litex.gen import LiteXModule

from litex.soc.integration.soc import SoCBusHandler, SoCRegion
from litex.soc.interconnect import wishbone

from litex_wr_nic.gateware.wr_cpu import (
    WR_CPU_MEMORY_ORIGIN,
    WR_CPU_MEMORY_SIZE,
    wr_cpu_word_bus,
)

# WR CPU Bus Tests ---------------------------------------------------------------------------------

@pytest.mark.parametrize("addressing", ["byte", "word"])
def test_wr_memory_remapping_preserves_high_address(addressing):
    dut     = LiteXModule()
    dut.bus = SoCBusHandler()
    master  = wishbone.Interface(data_width=32, address_width=32, addressing=addressing)
    memory  = wishbone.Interface(data_width=32, address_width=32, addressing="word")
    low     = wishbone.Interface.like(memory)
    region  = SoCRegion(origin=WR_CPU_MEMORY_ORIGIN, size=WR_CPU_MEMORY_SIZE)
    dut.bus.add_slave("wr_cpu_mem", memory, region)
    dut.bus.add_slave("low", low, SoCRegion(origin=0, size=WR_CPU_MEMORY_SIZE))
    dut.bus.add_master("wr_cpu", wr_cpu_word_bus(dut, master), region)
    dut.comb += [
        memory.ack.eq(memory.cyc & memory.stb),
        memory.dat_r.eq(0x12345678),
        low.ack.eq(low.cyc & low.stb),
        low.dat_r.eq(0xdeadbeef),
    ]

    def access(offset, write):
        yield master.adr.eq(offset if addressing == "byte" else offset // 4)
        yield master.we.eq(write)
        yield master.sel.eq(0b1010)
        yield master.dat_w.eq(0xcafebabe)
        yield master.cyc.eq(1)
        yield master.stb.eq(1)
        for _ in range(20):
            yield
            if (yield master.ack):
                assert (yield low.cyc) == 0, "WR memory access aliased into low SoC registers"
                assert (yield memory.cyc) == 1
                assert (yield memory.adr) == (WR_CPU_MEMORY_ORIGIN + offset) // 4
                assert (yield memory.we) == write
                assert (yield memory.sel) == 0b1010
                assert (yield memory.dat_w) == 0xcafebabe
                assert (yield master.dat_r) == 0x12345678
                break
        else:
            raise AssertionError("WR memory access stalled")
        yield master.cyc.eq(0)
        yield master.stb.eq(0)
        yield

    def generator():
        for offset in (0, 0x40, WR_CPU_MEMORY_SIZE - 4):
            yield from access(offset, 0)
            yield from access(offset, 1)

    run_simulation(dut, generator())
