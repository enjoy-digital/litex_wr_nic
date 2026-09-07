"""WR control/fabric must operate even while the PHY reference clock is stopped."""

from types import SimpleNamespace

from migen import ClockDomain, ClockSignal, Instance, Record, run_simulation
from migen.fhdl.structure import _Assign
from litex.gen import LiteXModule

from litex_wr_nic.gateware.soc import LiteXWRNICSoC


def test_wr_interfaces_without_phy_clock():
    dut = LiteXModule()
    dut.cd_sys = ClockDomain("sys")
    dut.platform = SimpleNamespace(device="xc7a50t")
    dut.bus = SimpleNamespace(add_slave=lambda **kwargs: None)
    LiteXWRNICSoC.add_wr_core(dut,
        cpu_firmware="unused.bram",
        sfp_pads=Record([(name, 1) for name in ("txp", "txn", "rxp", "rxn")]),
        sfp_i2c_pads=Record([("sda", 1), ("scl", 1)]))

    wb = dut.wb_slave_wr
    tx = dut.wrf_stream2wb
    rx = dut.wrf_wb2stream
    # Model only the WR core's bus responses. Exercise the actual LiteX-side
    # crossings and fabric adapters, with no edges on the PHY reference clock.
    dut.comb += [
        wb.ack.eq(wb.cyc & wb.stb),
        wb.dat_r.eq(0x12345678),
        tx.bus.ack.eq(tx.bus.cyc & tx.bus.stb),
    ]
    fragment = dut.get_fragment()
    core = [s for s in fragment.specials if isinstance(s, Instance)
            and s.of == "xwrc_board_litex_wr_nic_wrapper"]
    assert len(core) == 1
    fragment.specials.remove(core[0])

    received = []
    transmitted = []

    def host():
        host_wb = dut.wb_slave_sys
        yield host_wb.cyc.eq(1)
        yield host_wb.stb.eq(1)
        for _ in range(100):
            if (yield host_wb.ack):
                assert (yield host_wb.err) == 0
                assert (yield host_wb.dat_r) == 0x12345678
                break
            yield
        else:
            raise AssertionError("WR register access stalled with the PHY clock stopped")
        yield host_wb.cyc.eq(0)
        yield host_wb.stb.eq(0)

        yield tx.sink.valid.eq(1)
        for index, byte in enumerate([0x12, 0x34, 0x56, 0x78]):
            yield tx.sink.data.eq(byte)
            yield tx.sink.last.eq(index == 3)
            for _ in range(100):
                yield
                if (yield tx.sink.ready):
                    break
            else:
                raise AssertionError("Fabric transmit stream stalled")
        yield tx.sink.valid.eq(0)
        yield rx.source.ready.eq(1)
        yield
        for _ in range(150):
            if (yield rx.source.valid):
                received.append((yield rx.source.data))
            yield

    def wr_core():
        # The WR core itself originates fabric traffic in its system domain.
        yield rx.bus.cyc.eq(1)
        yield rx.bus.stb.eq(1)
        yield rx.bus.adr.eq(2)
        yield
        yield rx.bus.adr.eq(0)
        yield rx.bus.sel.eq(3)
        for word in [0xabcd, 0xef01]:
            yield rx.bus.dat_w.eq(word)
            yield
        yield rx.bus.cyc.eq(0)
        yield rx.bus.stb.eq(0)
        for _ in range(150):
            if (yield tx.bus.cyc) and (yield tx.bus.stb) and (yield tx.bus.adr) == 0:
                transmitted.append((yield tx.bus.dat_w))
            yield

    clocks = {"sys": 10, "wr_sys": 16}
    # Migen simulation needs explicit schedules for the clock aliases that
    # stream.ClockDomainCrossing creates to share resets across its FIFO.
    for statement in fragment.comb:
        if isinstance(statement, _Assign) and isinstance(statement.r, ClockSignal):
            if statement.r.cd in clocks:
                for domain in fragment.clock_domains:
                    if statement.l is domain.clk:
                        clocks[domain.name] = clocks[statement.r.cd]
    run_simulation(fragment, {"sys": host(), "wr_sys": wr_core()}, clocks=clocks)
    assert transmitted == [0x1234, 0x5678]
    assert received == [0xab, 0xcd, 0xef, 0x01]
