#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Whole-frame buffering and the actual LiteEth MAC path used by the bridge."""

import random
import zlib

from migen import ClockDomain, ResetInserter, run_simulation
from migen.sim import passive

from litex.gen import LiteXModule

from litex.soc.interconnect import stream

from liteeth.common import eth_phy_description

from litex_wr_nic.gateware.rgmii import WRPacketBuffer, WRRGMIIBridge


def send(endpoint, data, pauses=False, error_at=None, first=True):
    for index, byte in enumerate(data):
        if pauses and index % 7 == 0:
            yield endpoint.valid.eq(0)
            for _ in range(3):
                yield
        yield endpoint.valid.eq(1)
        yield endpoint.data.eq(byte)
        yield endpoint.first.eq(first and index == 0)
        yield endpoint.last.eq(index == len(data) - 1)
        if hasattr(endpoint, "last_be"):
            yield endpoint.last_be.eq(index == len(data) - 1)
            yield endpoint.error.eq(error_at is not None and index >= error_at)
        yield
        while not (yield endpoint.ready):
            yield
    yield endpoint.valid.eq(0)
    yield endpoint.last.eq(0)
    yield


def test_packet_buffer_drops_whole_frames_and_recovers():
    dut = WRPacketBuffer(depth=64, slots=2)
    received = []
    frame = []

    @passive
    def receiver():
        randomizer = random.Random(40)
        for _ in range(200):
            yield
        while True:
            yield dut.source.ready.eq(randomizer.randrange(4) != 0)
            yield
            if (yield dut.source.valid) and (yield dut.source.ready):
                frame.append((yield dut.source.data))
                if (yield dut.source.last):
                    received.append(bytes(frame))
                    frame.clear()

    def driver():
        # Two occupied slots must be retained while their consumer is stalled.
        yield from send(dut.sink, bytes(range(64)))
        yield from send(dut.sink, b"second")
        yield from send(dut.sink, b"full-buffer-drop")
        for _ in range(300):
            yield
        # Error and oversize discards must not consume a descriptor or replay
        # a partial packet as the prefix of the next good packet.
        yield from send(dut.sink, b"bad-frame", error_at=3)
        yield from send(dut.sink, bytes(range(65)), pauses=True)
        yield from send(dut.sink, b"after-errors", pauses=True)
        for _ in range(150):
            yield
        assert (yield dut.packets.status) == 3
        assert (yield dut.dropped.status) == 3

    run_simulation(dut, [driver(), receiver()])
    assert received == [bytes(range(64)), b"second", b"after-errors"]
    assert not frame


def test_packet_buffer_waits_for_last_then_streams_without_gaps():
    dut = WRPacketBuffer(depth=2048)
    data = bytes(index & 255 for index in range(1519))
    received = []
    complete = [False]

    @passive
    def receiver():
        yield dut.source.ready.eq(1)
        started = False
        while True:
            yield
            valid = (yield dut.source.valid)
            if started and len(received) != len(data):
                assert valid, "An RGMII frame cannot pause after transmission starts"
            if valid:
                assert complete[0], "A frame was exposed before its final byte arrived"
                started = True
                received.append((yield dut.source.data))
                assert bool((yield dut.source.last)) == (len(received) == len(data))

    def driver():
        yield from send(dut.sink, data, pauses=True)
        complete[0] = True
        for _ in range(len(data) + 100):
            yield

    run_simulation(dut, [driver(), receiver()])
    assert bytes(received) == data


def test_packet_buffer_reset_discards_an_incomplete_frame():
    dut = ResetInserter()(WRPacketBuffer(depth=64, with_first=True))
    received = []

    @passive
    def receiver():
        frame = []
        yield dut.source.ready.eq(1)
        while True:
            yield
            if (yield dut.source.valid):
                frame.append((yield dut.source.data))
                if (yield dut.source.last):
                    received.append(bytes(frame))
                    frame.clear()

    def driver():
        for index, byte in enumerate(b"prefix"):
            yield dut.sink.valid.eq(1)
            yield dut.sink.first.eq(index == 0)
            yield dut.sink.data.eq(byte)
            yield
        yield dut.sink.valid.eq(0)
        yield dut.reset.eq(1)
        yield
        yield
        yield dut.reset.eq(0)
        yield from send(dut.sink, b"tail", first=False)
        yield from send(dut.sink, b"new-frame")
        for _ in range(50):
            yield
        assert (yield dut.packets.status) == 1
        assert (yield dut.dropped.status) == 1

    run_simulation(dut, [driver(), receiver()])
    assert received == [b"new-frame"]


def test_packet_buffer_capture_reset_preserves_an_output_frame():
    dut = WRPacketBuffer(depth=64)
    received = []
    payload = bytes(range(64))

    def driver():
        yield from send(dut.sink, payload)
        yield dut.source.ready.eq(1)
        while len(received) < 10:
            yield
        yield dut.source.ready.eq(0)
        # The previous packet is partially delivered. Abort a new capture
        # while its consumer is stalled, then resume both directions.
        yield dut.sink.valid.eq(1)
        yield dut.sink.last.eq(0)
        for _ in range(5):
            yield
        yield dut.sink.valid.eq(0)
        yield dut.clear.eq(1)
        for _ in range(5):
            yield
        yield dut.clear.eq(0)
        yield dut.source.ready.eq(1)
        yield from send(dut.sink, b"after-reset")
        for _ in range(120):
            yield
        assert (yield dut.dropped.status) == 1

    @passive
    def receiver():
        while True:
            yield
            if (yield dut.source.valid) and (yield dut.source.ready):
                received.append(((yield dut.source.data), (yield dut.source.last)))

    run_simulation(dut, [driver(), receiver()])
    assert bytes(byte for byte, last in received) == payload + b"after-reset"
    assert [i for i, (byte, last) in enumerate(received) if last] == [63, 74]


class StreamPHY(LiteXModule):
    dw = 8
    def __init__(self):
        self.sink = stream.Endpoint(eth_phy_description(8))
        self.source = stream.Endpoint(eth_phy_description(8))


def wire_frame(payload):
    return b"\x55" * 7 + b"\xd5" + payload + zlib.crc32(payload).to_bytes(4, "little")


def test_bridge_mac_framing_crc_and_backpressure():
    dut = LiteXModule()
    dut.cd_sys = ClockDomain("sys")
    dut.cd_eth_tx = ClockDomain("eth_tx")
    dut.cd_eth_rx = ClockDomain("eth_rx")
    dut.phy = StreamPHY()
    wr_source = stream.Endpoint([("data", 8)], name="wr_source")
    wr_sink = stream.Endpoint([("data", 8)], name="wr_sink")
    dut.bridge = WRRGMIIBridge(dut.phy, wr_source, wr_sink)
    incoming = bytes(range(61))
    outgoing = bytes((index * 3) & 255 for index in range(1518))
    received, transmitted = [], []

    def wr_driver():
        yield from send(wr_source, outgoing, pauses=True)
        for _ in range(2500):
            yield

    @passive
    def wr_receiver():
        frame = []
        randomizer = random.Random(40)
        while True:
            yield wr_sink.ready.eq(randomizer.randrange(4) != 0)
            yield
            if (yield wr_sink.valid) and (yield wr_sink.ready):
                frame.append((yield wr_sink.data))
                if (yield wr_sink.last):
                    received.append(bytes(frame))
                    frame.clear()

    def phy_driver():
        for _ in range(20):
            yield
        frames = [
            (wire_frame(incoming), None),
            (wire_frame(incoming)[:-1] + bytes([wire_frame(incoming)[-1] ^ 1]), None),
            (wire_frame(incoming), 25),
            (wire_frame(b"runt"), None),
            (wire_frame(b"\xab" * 60), None),
        ]
        for data, error_at in frames:
            # Physical RX cannot honor backpressure.
            for index, byte in enumerate(data):
                yield dut.phy.source.valid.eq(1)
                yield dut.phy.source.data.eq(byte)
                yield dut.phy.source.last.eq(index == len(data) - 1)
                yield dut.phy.source.last_be.eq(index == len(data) - 1)
                yield dut.phy.source.error.eq(error_at is not None and index >= error_at)
                yield
            yield dut.phy.source.valid.eq(0)
            yield dut.phy.source.last.eq(0)
            yield dut.phy.source.error.eq(0)
            for _ in range(16):
                yield

    @passive
    def phy_receiver():
        yield dut.phy.sink.ready.eq(1)
        frame = []
        while True:
            yield
            if (yield dut.phy.sink.valid):
                frame.append((yield dut.phy.sink.data))
            elif frame:
                transmitted.append(bytes(frame))
                frame.clear()

    run_simulation(dut, {"sys": [wr_driver(), wr_receiver()],
        "eth_rx": phy_driver(), "eth_tx": phy_receiver()},
        clocks={"sys": 8, "rgmii_sys": 8, "eth_tx": (8, 2), "eth_rx": (8, 3)})
    assert received == [incoming, b"\xab" * 60]
    assert transmitted == [wire_frame(outgoing)]
