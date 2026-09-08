#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""The LiteX CPU adapter must run entirely in the WR system domain."""

from types import SimpleNamespace

from migen import ClockDomain, Signal, run_simulation

from litex.gen import LiteXModule

from litex.soc.interconnect import wishbone
from litex.soc.cores import cpu as litex_cpu

from litex_wr_nic.gateware.wr_cpu import WRLiteXCPU

# LiteX WR CPU Clock Test --------------------------------------------------------------------------

def test_litex_cpu_adapter_without_phy_clock(monkeypatch):
    class ModelCPU(LiteXModule):
        family     = "riscv"
        data_width = 32
        endianness = "little"

        def __init__(self, platform, variant):
            self.ibus      = wishbone.Interface(data_width=32, address_width=32, addressing="word")
            self.dbus      = wishbone.Interface.like(self.ibus)
            self.interrupt = Signal(32)
            self.reset     = Signal()
            self.heartbeat = Signal(16)
            self.sync += self.heartbeat.eq(self.heartbeat + 1)

        def set_reset_address(self, address):
            assert address == 0

    monkeypatch.setitem(litex_cpu.CPUS, "vexriscv", ModelCPU)
    dut           = LiteXModule()
    dut.cd_wr_sys = ClockDomain("wr_sys")
    dut.cd_wr     = ClockDomain("wr")
    dut.cd_sys    = ClockDomain("sys")
    dut.adapter   = adapter = WRLiteXCPU(SimpleNamespace(), "vexriscv")

    memory     = adapter.memory_bus
    peripheral = adapter.peripheral_bridge
    dut.comb += [
        memory.ack.eq(memory.cyc & memory.stb),
        memory.dat_r.eq(0x12345678),
        peripheral.ack.eq(peripheral.cyc & peripheral.stb),
        peripheral.dat_r.eq(0xabcdef01),
    ]

    def access(bus, address, expected):
        yield bus.adr.eq(address // 4)
        yield bus.cyc.eq(1)
        yield bus.stb.eq(1)
        for _ in range(30):
            yield
            if (yield bus.ack):
                assert (yield bus.dat_r) == expected
                break
        else:
            raise AssertionError(f"CPU access at {address:#x} needs a non-WR-system clock")
        yield bus.cyc.eq(0)
        yield bus.stb.eq(0)
        for _ in range(5):
            yield

    def cpu():
        # The initial arbiter grant is data; an instruction fetch forces it
        # to switch, then the data access forces it back again.
        yield from access(adapter.core.ibus, 0, 0x12345678)
        yield from access(adapter.core.dbus, 0x1000, 0x12345678)
        yield from access(adapter.core.dbus, 0x100020, 0xabcdef01)
        assert (yield adapter.core.heartbeat) > 0

    run_simulation(dut, {"wr_sys": cpu()}, clocks={"wr_sys": 16})
