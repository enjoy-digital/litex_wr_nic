#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.soc.interconnect import wishbone

from litex_wr_nic.gateware.wr_cpu import WRCPULocalMemory
from litex_wr_nic.gateware.wb_clock_crossing import WishboneClockCrossing

# Helpers ------------------------------------------------------------------------------------------

def _host_write(mem, word, value):
    yield mem.host.adr.eq(word)
    yield mem.host.dat_w.eq(value)
    yield mem.host.sel.eq(15)
    yield mem.host.we.eq(1)
    yield mem.host.cyc.eq(1)
    yield mem.host.stb.eq(1)
    yield
    while not (yield mem.host.ack):
        yield
    yield mem.host.cyc.eq(0)
    yield mem.host.stb.eq(0)
    yield mem.host.we.eq(0)
    yield


def _host_read(mem, word):
    yield mem.host.adr.eq(word)
    yield mem.host.cyc.eq(1)
    yield mem.host.stb.eq(1)
    yield
    while not (yield mem.host.ack):
        yield
    value = (yield mem.host.dat_r)
    yield mem.host.cyc.eq(0)
    yield mem.host.stb.eq(0)
    yield
    return value

# Local Memory Tests -------------------------------------------------------------------------------

def test_cpu_port_answers_one_request_per_cycle():
    mem = WRCPULocalMemory(size=1024, contents=[0x1000 + i for i in range(256)])

    def stimulus():
        # Back-to-back fetches: each is acknowledged exactly one cycle later.
        yield mem.cpu.cyc.eq(1)
        yield mem.cpu.stb.eq(1)
        yield mem.cpu.sel.eq(15)
        acks = []
        for word in range(8):
            yield mem.cpu.adr.eq(4*word)
            yield
            if (yield mem.cpu.ack):
                acks.append((yield mem.cpu.dat_r))
        yield mem.cpu.stb.eq(0)
        yield
        acks.append((yield mem.cpu.dat_r))
        assert (yield mem.cpu.ack) == 1
        yield
        assert (yield mem.cpu.ack) == 0
        assert acks == [0x1000 + i for i in range(8)]

    run_simulation(mem, stimulus())


def test_cpu_writes_honor_byte_selects_and_return_data():
    mem = WRCPULocalMemory(size=1024, contents=[0] * 256)

    def stimulus():
        yield mem.cpu.cyc.eq(1)
        yield mem.cpu.stb.eq(1)
        yield mem.cpu.adr.eq(0x10)
        yield mem.cpu.we.eq(1)
        yield mem.cpu.sel.eq(0b0011)
        yield mem.cpu.dat_w.eq(0xa5a5a5a5)
        yield
        yield mem.cpu.we.eq(0)
        yield mem.cpu.sel.eq(15)
        yield
        assert (yield mem.cpu.ack) == 1 # write acknowledged
        yield
        assert (yield mem.cpu.ack) == 1 # read acknowledged
        assert (yield mem.cpu.dat_r) == 0x0000a5a5
        yield mem.cpu.stb.eq(0)
        yield mem.cpu.cyc.eq(0)
        yield
        assert (yield from _host_read(mem, 4)) == 0x0000a5a5

    run_simulation(mem, stimulus())


def test_host_uses_idle_cycles_and_stalls_a_busy_cpu():
    mem = WRCPULocalMemory(size=1024, contents=[0] * 256, host_wait=4)

    def stimulus():
        # Idle CPU: the host is served without stalling anything.
        yield from _host_write(mem, 3, 0x12345678)
        assert (yield from _host_read(mem, 3)) == 0x12345678

        # Continuously fetching CPU: the host waits, then steals one cycle.
        yield mem.cpu.cyc.eq(1)
        yield mem.cpu.stb.eq(1)
        yield mem.cpu.sel.eq(15)
        yield mem.cpu.adr.eq(12)
        yield mem.host.adr.eq(3)
        yield mem.host.cyc.eq(1)
        yield mem.host.stb.eq(1)
        stalls = 0
        acked  = False
        for _ in range(12):
            yield
            stalls += (yield mem.stall)
            if (yield mem.host.ack):
                assert (yield mem.host.dat_r) == 0x12345678
                acked = True
                yield mem.host.cyc.eq(0)
                yield mem.host.stb.eq(0)
        assert acked
        assert stalls == 1
        # The CPU request issued during the stall was not acknowledged; the
        # remaining ones were.
        yield mem.cpu.stb.eq(0)
        yield
        yield
        assert (yield mem.cpu.dat_r) == 0x12345678

    run_simulation(mem, stimulus())

# Clock Crossing Stall Tests -----------------------------------------------------------------------

def test_crossing_captures_request_on_acceptance_and_stalls_until_response():
    dut    = Module()
    master = wishbone.Interface()
    target = wishbone.Interface()
    dut.submodules.bridge = bridge = WishboneClockCrossing(None,
        wb_from=master, cd_from="sys", wb_to=target, cd_to="sys", timeout_cycles=8)
    seen = Signal(32)
    dut.sync += If(target.cyc & target.stb, seen.eq(target.adr))
    delay = Signal(4)
    dut.sync += If(target.cyc, delay.eq(delay + 1)).Else(delay.eq(0))
    dut.comb += [
        target.ack.eq(target.cyc & target.stb & (delay == 2)),
        target.dat_r.eq(target.adr + 1),
    ]

    def stimulus():
        yield master.cyc.eq(1)
        yield master.stb.eq(1)
        yield master.sel.eq(15)
        yield master.adr.eq(0x10)
        yield
        # Presented and accepted at the coming edge: a pipelined master may
        # then present another address, which must neither be forwarded nor
        # lose the captured request.
        assert (yield bridge.stall) == 0
        yield master.adr.eq(0x20)
        yield
        for _ in range(10):
            if (yield master.ack):
                break
            assert (yield bridge.stall) == 1
            yield
        else:
            raise AssertionError("No response for the captured request")
        assert (yield master.dat_r) == 0x11
        assert (yield seen) == 0x10
        yield
        assert (yield bridge.stall) == 0

    run_simulation(dut, stimulus())
