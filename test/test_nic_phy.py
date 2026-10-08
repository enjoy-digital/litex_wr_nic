#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""The WR fabric adapter must preserve bytes through the LiteEth MAC converters."""

from types import SimpleNamespace

from migen import ClockDomain, run_simulation
from migen.sim import passive

from litex.gen import LiteXModule
from litex.soc.interconnect import stream

from liteeth.mac.core import LiteEthMACCore

from litex_wr_nic.gateware.nic.phy import LiteEthPHYWRGMII


def test_wr_phy_mac_loopback_with_partial_words_and_backpressure():
    dut = LiteXModule()
    dut.cd_sys = ClockDomain("sys")
    wr_source = stream.Endpoint([("data", 8)], name="wr_source")
    wr_sink = stream.Endpoint([("data", 8)], name="wr_sink")
    dut.phy = LiteEthPHYWRGMII(
        SimpleNamespace(sink=wr_sink), SimpleNamespace(source=wr_source))
    dut.mac = LiteEthMACCore(dut.phy, dw=32)
    dut.comb += dut.mac.source.connect(dut.mac.sink)
    packets = [bytes(range(length)) for length in (1, 3, 4, 5, 61, 64)]
    received, masks = [], []

    @passive
    def receiver():
        frame = []
        cycle = 0
        while True:
            yield wr_sink.ready.eq(cycle % 3 != 0)
            yield
            cycle += 1
            if (yield wr_sink.valid) and (yield wr_sink.ready):
                frame.append((yield wr_sink.data))
                if (yield wr_sink.last):
                    received.append(bytes(frame))
                    frame.clear()
            if (yield dut.mac.source.valid) and (yield dut.mac.source.ready):
                masks.append(((yield dut.mac.source.be), (yield dut.mac.source.last)))

    def driver():
        for number, payload in enumerate(packets):
            for index, byte in enumerate(payload):
                yield wr_source.valid.eq(1)
                yield wr_source.data.eq(byte)
                yield wr_source.last.eq(index == len(payload) - 1)
                yield
                for _ in range(100):
                    if (yield wr_source.ready):
                        break
                    yield
                else:
                    raise AssertionError("WR input stalled")
            yield wr_source.valid.eq(0)
            for _ in range(500):
                yield
                if len(received) == number + 1:
                    break
            else:
                raise AssertionError("MAC lost a WR frame")

    run_simulation(dut, [driver(), receiver()],
        clocks={"sys": 10, "eth_rx": 10, "eth_tx": 10})
    assert received == packets
    assert masks == [
        ((1 << min(4, len(payload) - offset)) - 1, int(offset + 4 >= len(payload)))
        for payload in packets for offset in range(0, len(payload), 4)
    ]
