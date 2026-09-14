# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from types import SimpleNamespace

from migen import *
from migen.sim import passive

from litex.soc.cores.code_8b10b import K
from litex_wr_nic.gateware.wr_phy import GW5WRPHY


class RawLoopback:
    def __init__(self, lane, valid=1, aligned=1):
        self.lane = lane
        self.valid = valid
        self.aligned = aligned

    def lower(self, instance):
        assert instance.of == 'GTR12_QUAD'
        n = self.lane
        model = Module()
        reset_n = instance.get_io(f'FABRIC_LN{n}_RSTN_I')
        model.comb += [
            instance.get_io(f'FABRIC_LN{n}_RXDATA_O').eq(instance.get_io(f'FABRIC_LN{n}_TXDATA_I')),
            instance.get_io(f'FABRIC_LN{n}_RX_VLD_OUT').eq(reset_n & self.valid),
            instance.get_io(f'LANE{n}_RX_IF_FIFO_EMPTY').eq(~(reset_n & self.valid)),
            instance.get_io(f'FABRIC_LANE{n}_CMU_OK_O').eq(reset_n),
            instance.get_io(f'FABRIC_LN{n}_PMA_RX_LOCK_O').eq(reset_n),
            instance.get_io(f'LANE{n}_ALIGN_LINK').eq(reset_n & self.aligned),
        ]
        return model


def test_raw_phy_preserves_data_and_control_symbols_on_both_lanes():
    for lane in (0, 1):
        platform = SimpleNamespace(devicename='GW5AST-138B',
            toolchain=SimpleNamespace(additional_tcl_commands=[]))
        dut = GW5WRPHY(platform, lane=lane)
        dut.cd_sys = ClockDomain('sys')
        seen = []
        expected = [(n, 0) for n in range(256)] + [(K(x, y), 1)
            for x, y in [(28, 5), (27, 7), (29, 7), (30, 7), (23, 7), (28, 7)]]

        @passive
        def receiver():
            while True:
                if not (yield dut.rx_error):
                    seen.append(((yield dut.rx_data), (yield dut.rx_k)))
                yield

        def sender():
            yield dut.tx_data.eq(K(28, 5))
            yield dut.tx_k.eq(1)
            for _ in range(20):
                yield
            for value, k in expected:
                yield dut.tx_data.eq(value)
                yield dut.tx_k.eq(k)
                yield
            yield dut.tx_data.eq(K(28, 5))
            yield dut.tx_k.eq(1)
            for _ in range(20):
                yield
            assert any(seen[i:i+len(expected)] == expected for i in range(len(seen)))

        run_simulation(dut, {'wr_phy_tx': sender(), 'wr_phy_rx': receiver()},
            clocks={'sys': 16, 'wr_phy_tx': 8, 'wr_phy_rx': 8},
            special_overrides={Instance: RawLoopback(lane)})


def test_receive_gaps_follow_decoder_latency_and_alignment_loss_recovers():
    platform = SimpleNamespace(devicename='GW5AST-138B',
        toolchain=SimpleNamespace(additional_tcl_commands=[]))
    dut = GW5WRPHY(platform)
    dut.cd_sys = ClockDomain('sys')
    valid = Signal(reset=1)
    aligned = Signal(reset=1)

    def stimulus():
        yield dut.tx_data.eq(K(28, 5))
        yield dut.tx_k.eq(1)
        for _ in range(20):
            yield
        assert (yield dut.ready)
        # Both qualifiers must follow the symbol through the decoder pipeline.
        for qualifier in (valid, aligned):
            history = [1, 1]
            for value in [1, 0, 1, 1, 0, 0, 1, 1, 1]:
                yield qualifier.eq(value)
                yield
                assert (yield dut.rx_error) == (not history.pop(0))
                history.append(value)
        yield aligned.eq(0)
        for _ in range(12):
            yield
        assert not (yield dut.ready)
        assert (yield dut.rx_error)
        yield aligned.eq(1)
        for _ in range(12):
            yield
        assert (yield dut.ready)
        assert not (yield dut.rx_error)
        yield dut.reset.eq(1)
        for _ in range(12):
            yield
        assert not (yield dut.ready)
        yield dut.reset.eq(0)
        for _ in range(20):
            yield
        assert (yield dut.ready)
        assert not (yield dut.rx_error)

    run_simulation(dut, {'wr_phy_rx': stimulus()},
        clocks={'sys': 16, 'wr_phy_tx': 8, 'wr_phy_rx': 8},
        special_overrides={Instance: RawLoopback(0, valid, aligned)})
