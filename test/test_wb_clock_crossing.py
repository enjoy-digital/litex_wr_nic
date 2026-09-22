#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import pytest

from migen import *

from litex.soc.interconnect import wishbone

from litex_wr_nic.gateware.wb_clock_crossing import WishboneClockCrossing


@pytest.mark.parametrize("response", ["ack", "err", "timeout"])
def test_same_clock_pipelined_requests_complete_and_recover(response):
    dut    = Module()
    master = wishbone.Interface()
    target = wishbone.Interface()
    dut.submodules.bridge = WishboneClockCrossing(None,
        wb_from=master, cd_from="sys", wb_to=target, cd_to="sys", timeout_cycles=8)
    delay = Signal(4)
    dut.sync += If(target.cyc, delay.eq(delay + 1)).Else(delay.eq(0))
    # The second request always succeeds, including after an error or timeout.
    success = (target.adr == 0x20) | (response == "ack")
    dut.comb += [
        target.ack.eq(target.cyc & target.stb & (delay == 3) & success),
        target.err.eq(target.cyc & target.stb & (delay == 3) & ~success & (response == "err")),
        target.dat_r.eq(0x12345678),
    ]

    def stimulus():
        for address in (0x10, 0x20):
            yield master.adr.eq(address)
            yield master.sel.eq(15)
            yield master.cyc.eq(1)
            yield master.stb.eq(1)
            yield
            # WR's pipelined memory master drops STB before receiving ACK.
            yield master.stb.eq(0)
            for _ in range(30):
                if (yield master.ack):
                    failed = address == 0x10 and response != "ack"
                    assert (yield master.err) == failed
                    expected = 0xffffffff if failed and response == "timeout" else 0x12345678
                    assert (yield master.dat_r) == expected
                    break
                yield
            else:
                raise AssertionError("Same-clock Wishbone request lost its response")
            yield master.cyc.eq(0)
            for _ in range(3):
                yield

    run_simulation(dut, stimulus())
