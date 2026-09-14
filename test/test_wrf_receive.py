#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""WR fabric packet boundaries must survive pauses and consumer stalls."""

from migen import ClockDomain, ClockSignal, run_simulation
from migen.fhdl.structure import _Assign
from migen.sim import passive

from litex.gen import LiteXModule

from litex_wr_nic.gateware.wrf_wb2stream import Wishbone2Stream


def test_fabric_receive_pauses_and_backpressure():
    dut = LiteXModule()
    dut.cd_sys = ClockDomain("sys")
    dut.cd_wr_sys = ClockDomain("wr_sys")
    dut.receiver = Wishbone2Stream(cd_from="wr_sys")
    bus, source = dut.receiver.bus, dut.receiver.source
    packets = [bytes(range(127)), b"abcd", b"odd"]
    received = []
    errors = []

    def sender():
        for number, payload in enumerate(packets):
            yield bus.cyc.eq(1)
            yield bus.stb.eq(1)
            yield bus.adr.eq(2)
            yield bus.dat_w.eq(0)
            yield bus.sel.eq(3)
            yield
            while not (yield bus.ack):
                yield
            for offset in range(0, len(payload), 2):
                if offset % 10 == 0:
                    yield bus.stb.eq(0)
                    for _ in range(3):
                        yield
                tail = payload[offset:offset + 2]
                yield bus.stb.eq(1)
                yield bus.adr.eq(0)
                yield bus.sel.eq(3 if len(tail) == 2 else 2)
                yield bus.dat_w.eq(int.from_bytes(tail.ljust(2, b"\0"), "big"))
                yield
                for _ in range(2000):
                    if (yield bus.ack):
                        break
                    yield
                else:
                    raise AssertionError("Fabric writer stalled permanently")
            # End-of-cycle need not immediately follow the final data beat.
            yield bus.stb.eq(0)
            for _ in range(5):
                yield
            if number == 2:
                yield bus.stb.eq(1)
                yield bus.adr.eq(2)
                yield bus.dat_w.eq(2)  # WR fabric status ERROR.
                yield
                while not (yield bus.ack):
                    yield
                yield bus.stb.eq(0)
            yield bus.cyc.eq(0)
            for _ in range(5):
                yield
        for _ in range(700):
            yield

    @passive
    def reader():
        frame = []
        cycle = 0
        while True:
            # Long enough to fill the small crossing FIFO and then force
            # ordinary gaps while draining it.
            yield source.ready.eq(cycle >= 100 and cycle % 5 != 0)
            yield
            cycle += 1
            if (yield source.valid) and (yield source.ready):
                assert bool((yield source.first)) == (not frame)
                frame.append((yield source.data))
                if (yield source.last):
                    received.append(bytes(frame))
                    errors.append(bool((yield dut.receiver.error)))
                    frame.clear()

    fragment = dut.get_fragment()
    clocks = {"sys": 10, "wr_sys": 16}
    for statement in fragment.comb:
        if isinstance(statement, _Assign) and isinstance(statement.r, ClockSignal):
            if statement.r.cd in clocks:
                for domain in fragment.clock_domains:
                    if statement.l is domain.clk:
                        clocks[domain.name] = clocks[statement.r.cd]
    run_simulation(fragment, {"sys": reader(), "wr_sys": sender()}, clocks=clocks)
    assert received == packets
    assert errors == [False, False, True]
