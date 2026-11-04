#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from types import SimpleNamespace

import pytest

from migen import run_simulation

from litex.soc.interconnect import csr_bus, wishbone

from litex_wr_nic.wr import WRClient, WRConsole, CPU_CSR, CPU_RESET, CPU_HALT, CPU_HALTED
from litex_wr_nic.gateware.wr_info import WRInfo, WR_INFO_MAGIC

# Host Models --------------------------------------------------------------------------------------

class Register:
    mode = "ro"

    def __init__(self, value=0):
        self.value = value

    def read(self):
        return self.value

    def write(self, value):
        self.value = value


class Bus:
    def __init__(self, external=False, info=False, cpu="urv"):
        self.mems = SimpleNamespace(wr_wb_slave=SimpleNamespace(base=0x20000000))
        if external:
            self.mems.wr_cpu_mem = SimpleNamespace(base=0x50000000, size=16)
        self.regs = SimpleNamespace()
        if info:
            for name, value in dict(
                magic         = WR_INFO_MAGIC,
                cpu_type      = int(cpu == "vexriscv"),
                memory_mode   = int(external),
                status        = 1,
                memory_size   = 128*1024,
                firmware_hash = 0x1234,
                reset_reason  = 0,
                reset_count   = 0,
            ).items():
                setattr(self.regs, "wr_info_" + name, Register(value))
        self.words      = {}
        self.operations = []
        self.corrupt    = False
        self.halts      = True
        self.address    = 0

    def read(self, address, length=None):
        if length is not None:
            return [self.read(address + 4*index) for index in range(length)]
        self.operations.append(("read", address))
        offset = address - self.mems.wr_wb_slave.base - CPU_CSR
        if offset == CPU_HALTED:
            return int(self.halts)
        if offset == 8:
            return self.words.get(self.address, 0) ^ int(self.corrupt)
        value = self.words.get(address, 0)
        if self.corrupt and address >= 0x50000000:
            value ^= 1
        return value

    def write(self, address, value):
        if isinstance(value, list):
            for index, word in enumerate(value):
                self.write(address + 4*index, word)
            return
        self.operations.append(("write", address, value))
        offset = address - self.mems.wr_wb_slave.base - CPU_CSR
        if offset == 4:
            self.address = value
        elif offset == 8:
            self.words[self.address] = value
        else:
            self.words[address] = value

# Management Tests ---------------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def no_delays(monkeypatch):
    monkeypatch.setattr("litex_wr_nic.wr.time.sleep", lambda seconds: None)


@pytest.mark.parametrize("external,cpu", [(False, "urv"), (True, "urv"), (True, "vexriscv")])
def test_firmware_load_verifies_and_uses_correct_byte_order(external, cpu):
    bus = Bus(external=external, info=True, cpu=cpu)
    wr = WRClient(bus)
    wr.size = 16
    data = b"\x01\x23\x45\x67\x89"
    wr.load_firmware(data, cpu)
    assert wr.read_cpu(CPU_RESET) == 0
    first = bus.words[0x50000000 if external else 0]
    assert first == (0x67452301 if external else 0x01234567)
    writes = [op for op in bus.operations if op[0] == "write"]
    base = bus.mems.wr_wb_slave.base + CPU_CSR
    if cpu == "urv":
        assert writes[:3] == [
            ("write", base + CPU_HALT, 1),
            ("write", base, 1),
            ("write", base + CPU_HALT, 0),
        ]
    else:
        assert all(op[1] != base + CPU_HALT for op in writes)
    assert writes[-1] == ("write", base, 0)


def test_bad_image_is_rejected_before_stopping_cpu():
    bus = Bus(external=True, info=True, cpu="vexriscv")
    wr = WRClient(bus)
    for data, cpu in [(b"bad", "urv"), (b"", "vexriscv"), (bytes(17), "vexriscv")]:
        with pytest.raises(ValueError):
            wr.load_firmware(data, cpu)
    assert not any(op[0] == "write" for op in bus.operations)


def test_corrupt_readback_leaves_cpu_in_reset():
    bus = Bus(external=True, info=True, cpu="vexriscv")
    bus.corrupt = True
    wr = WRClient(bus)
    with pytest.raises(RuntimeError, match="verification failed"):
        wr.load_firmware(b"abcd", "vexriscv")
    assert wr.read_cpu(CPU_RESET) == 1


def test_failed_halt_restores_debug_request_without_reset():
    bus = Bus()
    bus.halts = False
    wr = WRClient(bus, timeout=0)
    with pytest.raises(TimeoutError, match="did not halt"):
        wr.restart()
    assert wr.read_cpu(CPU_HALT) == 0
    assert wr.read_cpu(CPU_RESET) == 0


def test_old_external_image_requires_cpu_selection():
    bus = Bus(external=True)
    wr = WRClient(bus)
    with pytest.raises(ValueError, match="older image needs"):
        wr.restart()
    assert not any(op[0] == "write" for op in bus.operations)


def test_live_identity_rejects_stale_map_and_cpu_mismatch():
    bus = Bus(external=True, info=True, cpu="vexriscv")
    assert WRClient(bus).status()["cpu_type"] == "vexriscv"
    with pytest.raises(ValueError, match="disagrees"):
        WRClient(bus, cpu_type="urv")
    bus.regs.wr_info_magic.value = 0
    with pytest.raises(RuntimeError, match="signature"):
        WRClient(bus)


def test_console_restores_uart_selection_on_error():
    bus = SimpleNamespace(regs=SimpleNamespace(
        uart_xover_rxtx    = Register(),
        uart_xover_rxempty = Register(1),
        uart_xover_txfull  = Register(1),
        uart_control      = Register(2),
    ))
    console = WRConsole(bus, timeout=0)
    with pytest.raises(TimeoutError):
        with console.selected():
            assert bus.regs.uart_control.value == 1
            console.send(b"x")
    assert bus.regs.uart_control.value == 2


def test_live_reset_diagnostics_cross_clock_domains():
    host = wishbone.Interface(data_width=32, address_width=32, addressing="word")
    dut = WRInfo(host, "urv", False, 1, 0, 1)
    # Include the field packing used by the real SoC CSR bank.
    dut.csr_bank = csr_bus.CSRBank(dut.get_csrs(), address=0)

    def write_reset(value):
        yield host.adr.eq((0x20000000 + CPU_CSR) >> 2)
        yield host.dat_w.eq(value)
        yield host.sel.eq(15)
        yield host.we.eq(1)
        yield host.cyc.eq(1)
        yield host.stb.eq(1)
        yield host.ack.eq(1)
        yield
        yield host.cyc.eq(0)
        yield host.stb.eq(0)
        yield host.ack.eq(0)
        yield

    def wr():
        yield from write_reset(1)
        yield from write_reset(1) # Repeated assertion must not count twice.
        yield from write_reset(0)
        for _ in range(10):
            yield
        yield from write_reset(1)

    def sys():
        for _ in range(200):
            yield
        assert (yield dut.status.status) == 0b1011
        assert (yield dut.reset_reason.status) == 1
        assert (yield dut.reset_count.status) == 2

    run_simulation(dut, {"wr_sys": wr(), "sys": sys()}, clocks={"wr_sys": 16, "sys": 10})


def test_console_reads_only_buffered_bytes_in_a_fixed_burst():
    calls = []
    data = Register()
    data.addr = 0x100
    bus = SimpleNamespace(regs=SimpleNamespace(
        uart_xover_rxtx=data, uart_xover_rxempty=Register(), uart_xover_txfull=Register(),
        uart_rxlevel=Register(3), uart_rxoverflow=Register()),
        read=lambda address, length, burst: calls.append((address, length, burst)) or [65, 66, 67])
    assert WRConsole(bus).receive() == b"ABC"
    assert calls == [(0x100, 3, "fixed")]
    bus.regs.uart_rxoverflow.value = 1
    with pytest.raises(RuntimeError, match="overflow"):
        WRConsole(bus).receive()


def test_console_fifo_level_and_overflow(monkeypatch):
    from litex.gen import LiteXModule
    from litex.soc.interconnect import stream
    from litex_wr_nic.gateware.uart import UARTShared, UARTPads

    class PHY(LiteXModule):
        def __init__(self, *args, **kwargs):
            self.source = stream.Endpoint([("data", 8)])
            self.sink   = stream.Endpoint([("data", 8)])

    monkeypatch.setattr("litex_wr_nic.gateware.uart.UARTPHY", PHY)
    dut = UARTShared(UARTPads(), 125e6, crossover_rx_depth=8)

    def check():
        for value in range(9):
            yield dut.xover_phy.source.valid.eq(1)
            yield dut.xover_phy.source.data.eq(value)
            yield
        yield dut.xover_phy.source.valid.eq(0)
        for _ in range(3):
            yield
        assert (yield dut.rxlevel.status) == 9
        assert (yield dut.rxoverflow.status) == 0
        # A PHY cannot apply backpressure to the remote transmitter.
        yield dut.xover_phy.source.valid.eq(1)
        yield
        yield dut.xover_phy.source.valid.eq(0)
        yield
        assert (yield dut.rxoverflow.status) == 1
        for value in range(9):
            assert (yield dut.xover._rxtx.rd_data) == value
            yield dut.xover._rxtx.rd_stb.eq(1)
            yield
            yield dut.xover._rxtx.rd_stb.eq(0)
            yield
        assert (yield dut.rxlevel.status) == 0

    run_simulation(dut, check())


def test_profiled_image_and_host_boot_of_a_stopped_urv():
    from litex_wr_nic.wr_boot import build_boot_image
    bus = Bus(external=True, info=True, cpu="urv")
    bus.halts = False # The CPU is held by host boot and cannot enter debug.
    bus.regs.wr_cpu_boot_host_ready = Register(0)
    bus.regs.wr_cpu_boot_memory_ready = Register(1)
    wr = WRClient(bus)
    wr.load_firmware(build_boot_image(b"test", cpu_type="urv"))
    assert bus.regs.wr_cpu_boot_host_ready.read() == 1
    assert wr.read_cpu(CPU_RESET) == 0
    assert not any(op[0] == "read" and op[1] == bus.mems.wr_wb_slave.base + CPU_CSR + CPU_HALTED
        for op in bus.operations)


def test_host_boot_rejects_upload_before_ddr_initialization():
    bus = Bus(external=True, info=True, cpu="vexriscv")
    bus.regs.wr_cpu_boot_host_ready = Register(0)
    bus.regs.wr_cpu_boot_memory_ready = Register(0)
    with pytest.raises(RuntimeError, match="memory controller"):
        WRClient(bus).load_firmware(b"test", "vexriscv")
    assert not any(op[0] == "write" for op in bus.operations)


def test_upload_fences_writes_and_detects_memory_aliasing():
    class QueuedBus(Bus):
        def __init__(self, alias=False):
            super().__init__(external=True, info=True, cpu="vexriscv")
            self.mems.wr_cpu_mem.size = 512
            self.queued = 0
            self.alias = alias

        def write(self, address, value):
            if address >= 0x50000000 and not isinstance(value, list):
                self.queued += 1
                assert self.queued <= 16, "Host queued writes without waiting for the transport"
                if self.alias:
                    address = 0x50000000 + (address - 0x50000000) % 64
            super().write(address, value)

        def read(self, address, length=None):
            self.queued = 0
            if self.alias and length is None and address >= 0x50000000:
                address = 0x50000000 + (address - 0x50000000) % 64
            return super().read(address, length)

    image = bytes(range(256)) + bytes(256)
    WRClient(QueuedBus()).load_firmware(image, "vexriscv")
    bus = QueuedBus(alias=True)
    with pytest.raises(RuntimeError, match="verification failed"):
        WRClient(bus).load_firmware(image, "vexriscv")
    assert WRClient(bus).read_cpu(CPU_RESET) == 1


def test_time_snapshot_command_and_stopped_clock_timeout():
    bus = Bus(info=True)
    fields = dict(capture=0, done=1, seconds=123, cycles=456, time_valid=1, link_up=1, state=1)
    for name, value in fields.items():
        setattr(bus.regs, "wr_time_" + name, Register(value))
    wr = WRClient(bus, timeout=0)
    assert wr.read_time() == dict(seconds=123, cycles=456, time_valid=1, link_up=1, state=1)
    assert bus.regs.wr_time_capture.read() == 1
    bus.regs.wr_time_done.value = 0
    with pytest.raises(TimeoutError, match="reference clock"):
        wr.read_time()
