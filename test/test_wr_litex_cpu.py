#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import pytest

from migen.sim import run_simulation

from litex.soc.interconnect import wishbone

from litex_wr_nic.gateware.wr_cpu import (
    WRCPUInterconnect,
    WRCPUPeripheralBridge,
    resolve_wr_cpu_variant,
    validate_wr_cpu_config,
    wr_cpu_firmware_filename,
)

# Helpers ------------------------------------------------------------------------------------------

def _make_interconnect():
    ibus = wishbone.Interface(data_width=32, address_width=32, addressing="word")
    dbus = wishbone.Interface(data_width=32, address_width=32, addressing="word")
    return WRCPUInterconnect(ibus, dbus), ibus, dbus

# LiteX WR CPU Tests -------------------------------------------------------------------------------

def test_wr_cpu_configuration_validation():
    assert resolve_wr_cpu_variant("urv") is None
    assert resolve_wr_cpu_variant("vexriscv") == "lite"
    assert validate_wr_cpu_config("vexriscv", "lite", "integrated") == "lite"
    assert validate_wr_cpu_config("vexriscv", "lite", "hyperram") == "lite"
    assert wr_cpu_firmware_filename("urv", "bin") == "spec_a7_wrc.bin"
    assert wr_cpu_firmware_filename("vexriscv", "boot") == "spec_a7_wrc_vexriscv.boot"

    with pytest.raises(ValueError, match="requires integrated or HyperRAM"):
        validate_wr_cpu_config("vexriscv", memory="private")
    with pytest.raises(ValueError, match="does not accept"):
        resolve_wr_cpu_variant("urv", "lite")
    with pytest.raises(ValueError, match="Unsupported vexriscv"):
        resolve_wr_cpu_variant("vexriscv", "standard")
    with pytest.raises(ValueError, match="Unsupported WR CPU type"):
        resolve_wr_cpu_variant("serv")


@pytest.mark.parametrize("byte_address", [0x0000_0000, 0x000f_fffc])
def test_data_low_memory_routing(byte_address):
    dut, _, dbus = _make_interconnect()

    def generator():
        yield dbus.adr.eq(byte_address // 4)
        yield dbus.cyc.eq(1)
        yield dbus.stb.eq(1)
        yield dbus.sel.eq(0b1010)
        yield dbus.we.eq(1)
        yield dbus.dat_w.eq(0x1234_5678)
        yield
        assert (yield dut.memory_bus.cyc) == 1
        assert (yield dut.peripheral_bus.cyc) == 0
        assert (yield dut.memory_bus.adr) == byte_address // 4
        assert (yield dut.memory_bus.sel) == 0b1010
        assert (yield dut.memory_bus.dat_w) == 0x1234_5678
        yield dut.memory_bus.ack.eq(1)
        yield
        assert (yield dbus.ack) == 1

    run_simulation(dut, generator())


@pytest.mark.parametrize("byte_address", [0x0010_0000, 0xffff_fffc])
def test_data_peripheral_routing(byte_address):
    dut, _, dbus = _make_interconnect()

    def generator():
        yield dbus.adr.eq(byte_address // 4)
        yield dbus.cyc.eq(1)
        yield dbus.stb.eq(1)
        yield
        assert (yield dut.memory_bus.cyc) == 0
        assert (yield dut.peripheral_bus.cyc) == 1
        assert (yield dut.peripheral_bus.adr) == byte_address // 4
        yield dut.peripheral_bus.err.eq(1)
        yield
        assert (yield dbus.err) == 1

    run_simulation(dut, generator())


def test_instruction_bus_always_uses_memory():
    dut, ibus, _ = _make_interconnect()

    def generator():
        yield ibus.adr.eq(0x0010_0000 // 4)
        yield ibus.cyc.eq(1)
        yield ibus.stb.eq(1)
        # Allow the round-robin arbiter to select its only requester.
        yield
        yield
        assert (yield dut.memory_bus.cyc) == 1
        assert (yield dut.peripheral_bus.cyc) == 0
        yield dut.memory_bus.dat_r.eq(0x0000_0013)
        yield dut.memory_bus.ack.eq(1)
        yield
        assert (yield ibus.ack) == 1
        assert (yield ibus.dat_r) == 0x0000_0013

    run_simulation(dut, generator())


def test_data_request_has_initial_arbitration_priority():
    dut, ibus, dbus = _make_interconnect()

    def generator():
        yield ibus.adr.eq(4)
        yield ibus.cyc.eq(1)
        yield ibus.stb.eq(1)
        yield dbus.adr.eq(8)
        yield dbus.cyc.eq(1)
        yield dbus.stb.eq(1)
        yield
        assert (yield dut.memory_bus.adr) == 8
        yield dut.memory_bus.ack.eq(1)
        yield
        assert (yield dbus.ack) == 1
        assert (yield ibus.ack) == 0

    run_simulation(dut, generator())


def test_peripheral_bridge_honors_stall_and_completes_error():
    bus = wishbone.Interface(data_width=32, address_width=32, addressing="word")
    dut = WRCPUPeripheralBridge(bus)

    def generator():
        yield dut.stall.eq(1)
        yield bus.cyc.eq(1)
        yield bus.stb.eq(1)
        yield bus.we.eq(1)
        yield bus.adr.eq(0x0010_0020 // 4)
        yield bus.sel.eq(0b0101)
        yield bus.dat_w.eq(0x89ab_cdef)
        yield
        yield
        assert (yield dut.cyc) == 1
        assert (yield dut.stb) == 1
        assert (yield dut.adr) == 0x0010_0020 // 4
        assert (yield dut.sel) == 0b0101
        assert (yield dut.dat_w) == 0x89ab_cdef

        # Keep STB asserted until the pipelined WR bus accepts the request.
        yield
        assert (yield dut.stb) == 1
        yield dut.stall.eq(0)
        yield
        yield
        assert (yield dut.cyc) == 1
        assert (yield dut.stb) == 0

        # LiteX VexRiscv ignores ERR, so an error is returned with ACK too.
        yield dut.dat_r.eq(0xdead_beef)
        yield dut.err.eq(1)
        yield
        yield
        assert (yield bus.ack) == 1
        assert (yield bus.err) == 1
        assert (yield bus.dat_r) == 0
        yield dut.err.eq(0)
        yield
        assert (yield bus.ack) == 0

        # The CPU holds its classic request through ACK. Do not launch a
        # duplicate WR transaction until it releases CYC/STB.
        yield
        assert (yield dut.cyc) == 0
        yield bus.cyc.eq(0)
        yield bus.stb.eq(0)
        yield

        # A later successful access returns the WR response data.
        yield bus.cyc.eq(1)
        yield bus.stb.eq(1)
        yield bus.we.eq(0)
        yield bus.adr.eq(0x0010_0040 // 4)
        yield
        yield
        yield dut.dat_r.eq(0x1234_5678)
        yield dut.ack.eq(1)
        yield
        yield
        assert (yield bus.ack) == 1
        assert (yield bus.err) == 0
        assert (yield bus.dat_r) == 0x1234_5678

    run_simulation(dut, generator())
