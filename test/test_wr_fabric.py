#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random

from migen import *
from migen.fhdl.structure import _Assign

from litex.gen import LiteXModule

from litex_wr_nic.gateware.wrf_stream2wb import Stream2Wishbone
from litex_wr_nic.gateware.wrf_wb2stream import Wishbone2Stream


def simulate(dut, generators):
    dut.cd_sys = ClockDomain("sys")
    dut.cd_wr = ClockDomain("wr")
    fragment = dut.get_fragment()
    clocks = {"sys": 8, "wr": 16}
    for statement in fragment.comb:
        if isinstance(statement, _Assign) and isinstance(statement.r, ClockSignal):
            if statement.r.cd in clocks:
                for domain in fragment.clock_domains:
                    if statement.l is domain.clk:
                        clocks[domain.name] = clocks[statement.r.cd]
    run_simulation(fragment, generators, clocks=clocks)


def send_packets(sink, packets, error_frames=()):
    for _ in range(10):
        yield
    for frame, packet in enumerate(packets):
        for index, byte in enumerate(packet):
            yield sink.valid.eq(1)
            yield sink.data.eq(byte)
            yield sink.first.eq(index == 0)
            yield sink.last.eq(index == len(packet) - 1)
            yield sink.error.eq(frame in error_frames and index == 0)
            for _ in range(5000):
                yield
                if (yield sink.ready):
                    break
            else:
                raise AssertionError("TX stream did not accept data")
        yield sink.valid.eq(0)
        yield


def test_packet_loopback_with_backpressure_odd_lengths_and_errors():
    dut = LiteXModule()
    dut.tx = Stream2Wishbone(depth=8)
    dut.rx = Wishbone2Stream(depth=8)
    dut.comb += dut.tx.bus.connect(dut.rx.bus)
    packets = [bytes((index + length) % 256 for index in range(length))
        for length in (1, 2, 3, 7, 16, 31, 128, 255)]
    received = []
    rng = random.Random(0x5742)

    def consumer():
        packet = bytearray()
        for tick in range(15000):
            ready = tick > 200 and rng.randrange(4) != 0
            yield dut.rx.source.ready.eq(ready)
            yield
            if (yield dut.rx.source.valid) and (yield dut.rx.source.ready):
                assert bool((yield dut.rx.source.first)) == (len(packet) == 0)
                packet.append((yield dut.rx.source.data))
                if (yield dut.rx.source.last):
                    received.append((bytes(packet), (yield dut.rx.source.error)))
                    packet.clear()
                    if len(received) == len(packets):
                        return
        raise AssertionError(("Packet loopback did not complete", received, bytes(packet)))

    simulate(dut, {"sys": [send_packets(dut.tx.sink, packets, error_frames=(1, 5)), consumer()]})
    assert received == [(packet, int(index in (1, 5))) for index, packet in enumerate(packets)]


def test_tx_retires_delayed_responses_and_keeps_request_stable_under_stall():
    dut = Stream2Wishbone(depth=16, max_pending=8)
    packets = [bytes(range(48)), b"abc"]
    frames = []
    accepted_ticks = []

    def fabric():
        responses = []
        words = []
        previous_cyc = 0
        stalled_request = None
        for tick in range(5000):
            cyc = yield dut.bus.cyc
            stb = yield dut.bus.stb
            stall = yield dut.bus.stall
            request = ((yield dut.bus.adr), (yield dut.bus.dat_w), (yield dut.bus.sel))
            if stalled_request is not None and cyc:
                assert stb and request == stalled_request
            stalled_request = request if cyc and stb and stall else None
            if cyc and stb and not stall:
                words.append(request)
                responses.append(tick + 4)
                accepted_ticks.append(tick)
            if previous_cyc and not cyc:
                assert not responses, "CYC dropped with outstanding responses"
                frames.append(words)
                words = []
                if len(frames) == 2:
                    return
            previous_cyc = cyc
            # Responses describe earlier accepted words and can coincide with
            # STALL on a new word. Do not equate ACK with acceptance.
            ack = bool(responses and responses[0] <= tick + 1)
            if ack:
                responses.pop(0)
            yield dut.bus.ack.eq(ack)
            yield dut.bus.stall.eq(tick < 70 or tick % 7 in (2, 3))
            yield
        raise AssertionError("TX did not complete")

    simulate(dut, {"sys": send_packets(dut.sink, packets), "wr": fabric()})
    for packet, frame in zip(packets, frames):
        assert frame[0] == (2, 0x0200, 3)
        data = bytearray()
        for address, value, sel in frame[1:]:
            assert address == 0
            data.append(value >> 8)
            if sel == 3:
                data.append(value & 255)
        assert bytes(data) == packet
    assert any(second - first == 1 for first, second in zip(accepted_ticks, accepted_ticks[1:]))


def test_rx_retains_end_marker_while_full_and_marks_trailing_status_error():
    dut = Wishbone2Stream(depth=4)
    received = []

    def master():
        for _ in range(10):
            yield
        yield dut.bus.cyc.eq(1)
        yield dut.bus.we.eq(1)
        yield dut.bus.stb.eq(1)
        for address, value, sel in [(2, 0x0200, 3), (0, 0x1234, 3), (0, 0x5600, 2),
            (1, 0xaaaa, 3), (2, 0x0202, 3)]:
            yield dut.bus.adr.eq(address)
            yield dut.bus.dat_w.eq(value)
            yield dut.bus.sel.eq(sel)
            for _ in range(500):
                yield
                if not (yield dut.bus.stall):
                    break
            else:
                raise AssertionError("RX remained stalled")
        yield dut.bus.cyc.eq(0)
        yield dut.bus.stb.eq(0)
        for _ in range(300):
            yield

    def consumer():
        for tick in range(600):
            yield dut.source.ready.eq(tick > 150)
            yield
            if (yield dut.source.valid) and (yield dut.source.ready):
                received.append(((yield dut.source.data), (yield dut.source.first),
                    (yield dut.source.last), (yield dut.source.error)))

    simulate(dut, {"wr": master(), "sys": consumer()})
    assert received == [(0x12, 1, 0, 0), (0x34, 0, 0, 0), (0x56, 0, 1, 1)]


def test_tx_aborts_error_and_retry_then_starts_a_clean_packet():
    for response in ("err", "rty"):
        dut = Stream2Wishbone(depth=8)
        packets = [bytes(range(32)), b"good"]
        frames = []

        def fabric():
            failed = False
            previous_cyc = 0
            words = []
            for _ in range(5000):
                cyc = yield dut.bus.cyc
                stb = yield dut.bus.stb
                yield dut.bus.ack.eq(0)
                yield getattr(dut.bus, response).eq(0)
                if cyc and stb:
                    words.append(((yield dut.bus.adr), (yield dut.bus.dat_w), (yield dut.bus.sel)))
                    if not failed and len(words) == 3:
                        yield getattr(dut.bus, response).eq(1)
                        failed = True
                    else:
                        yield dut.bus.ack.eq(1)
                if previous_cyc and not cyc:
                    frames.append(words)
                    words = []
                    if len(frames) == 2:
                        return
                previous_cyc = cyc
                yield
            raise AssertionError("TX did not recover")

        simulate(dut, {"sys": send_packets(dut.sink, packets), "wr": fabric()})
        assert frames[-1] == [(2, 0x0200, 3), (0, 0x676f, 3), (0, 0x6f64, 3)]


def test_reset_flushes_partial_packet_state_on_both_sides():
    dut = LiteXModule()
    dut.tx = Stream2Wishbone(depth=4)
    dut.rx = Wishbone2Stream(depth=4)
    dut.comb += dut.tx.bus.connect(dut.rx.bus)
    after_reset = {"active": False}
    received = []

    def producer():
        for _ in range(10):
            yield
        for index in range(12):
            yield dut.tx.sink.valid.eq(1)
            yield dut.tx.sink.first.eq(index == 0)
            yield dut.tx.sink.last.eq(0)
            yield dut.tx.sink.data.eq(0xff)
            yield dut.tx.sink.error.eq(1)
            while True:
                yield
                if (yield dut.tx.sink.ready):
                    break
        yield dut.tx.sink.valid.eq(0)
        for _ in range(30):
            yield
        yield dut.cd_wr.rst.eq(1)
        for _ in range(20):
            yield
        yield dut.cd_wr.rst.eq(0)
        for _ in range(20):
            yield
        after_reset["active"] = True
        yield from send_packets(dut.tx.sink, [b"new"])

    def consumer():
        yield dut.rx.source.ready.eq(1)
        for _ in range(1500):
            yield
            if after_reset["active"] and (yield dut.rx.source.valid) and (yield dut.rx.source.ready):
                received.append(((yield dut.rx.source.data), (yield dut.rx.source.first),
                    (yield dut.rx.source.last), (yield dut.rx.source.error)))

    simulate(dut, {"sys": [producer(), consumer()]})
    assert received == [(ord("n"), 1, 0, 0), (ord("e"), 0, 0, 0), (ord("w"), 0, 1, 0)]


def test_rx_reports_a_master_that_aborts_a_stalled_word():
    dut = Wishbone2Stream(depth=4)
    dropped = {"done": False, "accepted": []}
    received = []

    def master():
        for _ in range(10):
            yield
        yield dut.bus.cyc.eq(1)
        yield dut.bus.we.eq(1)
        yield dut.bus.stb.eq(1)
        yield dut.bus.adr.eq(2)
        yield dut.bus.dat_w.eq(0x0200)
        yield dut.bus.sel.eq(3)
        while True:
            yield
            if not (yield dut.bus.stall):
                break
        yield dut.bus.adr.eq(0)
        for index in range(64):
            yield dut.bus.dat_w.eq(index)
            yield
            if (yield dut.bus.stall):
                # Deliberately violate the protocol by abandoning this word.
                for _ in range(3):
                    yield
                yield dut.bus.cyc.eq(0)
                yield dut.bus.stb.eq(0)
                dropped["done"] = True
                break
            dropped["accepted"].extend([0, index])
        else:
            raise AssertionError("RX FIFO never applied backpressure")
        for _ in range(300):
            yield
        assert (yield dut.stats.overflow) == 1
        assert (yield dut.stats.errors) == 1

    def consumer():
        for _ in range(1000):
            yield dut.source.ready.eq(dropped["done"])
            yield
            if (yield dut.source.valid) and (yield dut.source.ready):
                received.append(((yield dut.source.data), (yield dut.source.last), (yield dut.source.error)))

    simulate(dut, {"wr": master(), "sys": consumer()})
    assert [data for data, _, _ in received] == dropped["accepted"]
    assert received[-1][1:] == (1, 1)
