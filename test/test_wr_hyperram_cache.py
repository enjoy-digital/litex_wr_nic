#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.fhdl.simplify import FullMemoryWE

from litex.gen import LiteXModule
from litex.soc.interconnect import wishbone


def test_local_cache_addresses_preserve_region_data_across_eviction():
    dut = LiteXModule()
    host = wishbone.Interface(data_width=32, address_width=32, addressing="word")
    local = wishbone.Interface(data_width=32, address_width=17, addressing="word")
    memory = wishbone.Interface(data_width=32, address_width=17, addressing="word")
    dut.comb += host.connect(local)
    dut.cache = FullMemoryWE()(wishbone.Cache(cachesize=32, master=local, slave=memory))
    contents = {}
    # Repeatedly collide cache lines, including the last word of the region.
    offsets = [base + word*4 for base in (0, 0x2000, 0x10000, 0x1ff80) for word in range(32)]
    expected = {offset: (offset * 977) ^ 0xa55a5aa5 for offset in offsets}

    @passive
    def ram():
        while True:
            if (yield memory.cyc) and (yield memory.stb):
                address = yield memory.adr
                assert 0 <= address < 128*1024//4
                data = yield memory.dat_w
                write = yield memory.we
                for _ in range(3):
                    yield
                if write:
                    assert (yield memory.sel) == 15
                    contents[address] = data
                yield memory.dat_r.eq(contents.get(address, 0))
                yield memory.ack.eq(1)
                yield
                yield memory.ack.eq(0)
                yield
            else:
                yield

    def check():
        for offset, value in expected.items():
            yield from host.write((0x40000000 + offset)//4, value)
        for offset, value in expected.items():
            assert (yield from host.read((0x40000000 + offset)//4)) == value
        # Partial writes retain the other byte lanes through an eviction.
        yield from host.write(0x40000000//4, 0x00112200, sel=6)
        yield from host.read((0x40000000 + 0x2000)//4)
        value = yield from host.read(0x40000000//4)
        assert value == (expected[0] & 0xff0000ff) | 0x00112200
    run_simulation(dut, [check(), ram()])
