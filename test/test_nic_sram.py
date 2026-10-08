#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Exercise the PCIe SRAM with LiteEth's per-transfer byte masks."""

import pytest

from migen import run_simulation
from migen.sim import passive

from litex_wr_nic.gateware.nic.sram import LiteEthMACSRAMReader, LiteEthMACSRAMWriter


def send(sink, payload, word_bytes, errors=None):
    for beat, offset in enumerate(range(0, len(payload), word_bytes)):
        tail = payload[offset:offset + word_bytes]
        yield sink.valid.eq(1)
        yield sink.data.eq(int.from_bytes(tail, "little"))
        yield sink.be.eq((1 << len(tail)) - 1)
        yield sink.last.eq(offset + word_bytes >= len(payload))
        yield sink.error.eq((errors or {}).get(beat, 0))
        yield
        assert (yield sink.ready)
        # A bubble must not affect the packet's length or error state.
        yield sink.valid.eq(0)
        yield sink.error.eq((1 << word_bytes) - 1)
        yield
    yield sink.error.eq(0)


@pytest.mark.parametrize("dw", [8, 16, 32, 64])
@pytest.mark.parametrize("endianness", ["big", "little"])
def test_writer_pcie_lengths_and_memory(dw, endianness):
    dut = LiteEthMACSRAMWriter(dw, depth=64, endianness=endianness)
    dut.specials += dut.mems
    word_bytes = dw // 8

    def bench():
        yield dut._enable.storage.eq(1)
        yield dut._pcie_host_addrs.storage.eq((0x1000 << 32) | 0x2000)
        # Single-word, aligned and every possible final-word byte count.
        for number, length in enumerate(range(1, 3 * word_bytes + 1)):
            payload = bytes((index + number) & 255 for index in range(length))
            yield from send(dut.sink, payload, word_bytes)
            for _ in range(30):
                yield
                if (yield dut.start):
                    break
            else:
                raise AssertionError("Missing RX DMA request")
            assert (yield dut._length.status) == length
            assert (yield dut.pcie_host_addr) == 0x1000
            assert (yield dut._pending_length.status) >> 32 == length
            slot = (yield dut._slot.status)
            assert slot == number % 2
            for address, offset in enumerate(range(0, length, word_bytes)):
                word = payload[offset:offset + word_bytes].ljust(word_bytes, b"\0")
                assert (yield dut.mems[slot][address]) == int.from_bytes(word, endianness)
            yield dut.ready.eq(1)
            yield
            yield dut.ready.eq(0)
            for _ in range(3):
                yield
            assert (yield dut._pending_slots.status) == 1
            yield dut._pending_clear.storage.eq(1)
            yield dut._pending_clear.re.eq(1)
            yield
            yield dut._pending_clear.re.eq(0)
            yield

    run_simulation(dut, bench())


@pytest.mark.parametrize("dw", [8, 16, 32, 64])
def test_writer_qualifies_errors_on_every_beat_and_recovers(dw):
    dut = LiteEthMACSRAMWriter(dw, depth=64, with_eth_pcie=False)
    dut.specials += dut.mems
    word_bytes = dw // 8
    payload = bytes(range(2 * word_bytes + 1))
    cases = [({0: 1}, False), ({1: 1}, False), ({2: 1}, False), ({}, True)]
    if word_bytes > 1:
        # Only lane 0 of the final word is valid; errors in padding are ignored.
        cases.append(({2: 1 << (word_bytes - 1)}, True))

    def bench():
        for errors, accepted in cases:
            yield from send(dut.sink, payload, word_bytes, errors)
            for _ in range(10):
                yield
            assert bool((yield dut.ev.available.pending)) == accepted
            if accepted:
                assert (yield dut._length.status) == len(payload)
                yield dut.ev.pending.wr_data.eq(1)
                yield dut.ev.pending.wr_stb.eq(1)
                yield
                yield dut.ev.pending.wr_stb.eq(0)
                yield

    run_simulation(dut, bench())


@pytest.mark.parametrize("dw", [8, 16, 32, 64])
@pytest.mark.parametrize("endianness", ["big", "little"])
def test_reader_pcie_masks_data_and_backpressure(dw, endianness):
    dut = LiteEthMACSRAMReader(dw, depth=64, endianness=endianness)
    dut.specials += dut.mems
    word_bytes = dw // 8
    received = []

    @passive
    def receiver():
        cycle = 0
        stalled = None
        while True:
            yield dut.source.ready.eq(cycle % 4 == 0)
            yield
            cycle += 1
            valid = (yield dut.source.valid)
            beat = ((yield dut.source.data), (yield dut.source.be), (yield dut.source.last))
            if stalled is not None:
                assert valid and beat == stalled
            stalled = beat if valid and not (yield dut.source.ready) else None
            if valid and (yield dut.source.ready):
                assert (yield dut.source.error) == 0
                received.append(beat)

    def bench():
        yield dut._pcie_host_addrs.storage.eq((0x1000 << 32) | 0x2000)
        for number, length in enumerate(range(1, 3 * word_bytes + 1)):
            payload = bytes((index + number) & 255 for index in range(length))
            slot = number % 2
            expected = []
            for address, offset in enumerate(range(0, length, word_bytes)):
                tail = payload[offset:offset + word_bytes]
                word = tail.ljust(word_bytes, b"\0")
                yield dut.mems[slot][address].eq(int.from_bytes(word, endianness))
                expected.append((int.from_bytes(word, "little"), (1 << len(tail)) - 1,
                    int(offset + word_bytes >= length)))
            received.clear()
            yield dut._slot.storage.eq(slot)
            yield dut._length.storage.eq(length)
            yield dut._start.re.eq(1)
            yield
            yield dut._start.re.eq(0)
            for _ in range(30):
                yield
                if (yield dut.start):
                    break
            else:
                raise AssertionError("Missing TX DMA request")
            assert (yield dut.pcie_host_addr) == (0x1000, 0x2000)[slot]
            for _ in range(3):
                yield
                assert not (yield dut.source.valid)
            yield dut.ready.eq(1)
            yield
            yield dut.ready.eq(0)
            for _ in range(100):
                yield
                if (yield dut._pending_slots.status) & (1 << slot):
                    break
            else:
                raise AssertionError("Missing TX completion")
            assert received == expected
            yield dut._pending_clear.storage.eq(1 << slot)
            yield dut._pending_clear.re.eq(1)
            yield
            yield dut._pending_clear.re.eq(0)
            yield

    run_simulation(dut, [bench(), receiver()])
