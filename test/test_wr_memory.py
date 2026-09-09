#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from types import SimpleNamespace

import pytest

from migen import Signal, run_simulation

from litex.gen import LiteXModule
from litex.soc.integration.soc import SoCRegion

from litex_wr_nic.gateware.wr_memory import WRCPUHostBoot, add_wr_cpu_memory, resolve_wr_boot

# Memory and Boot Policy Tests ---------------------------------------------------------------------

def test_host_boot_waits_for_host_and_memory_controller():
    ready = Signal()
    dut   = WRCPUHostBoot(ready)

    def check():
        assert (yield dut.ready) == 0
        yield dut._host_ready.storage.eq(1)
        yield
        assert (yield dut.ready) == 0
        yield ready.eq(1)
        yield
        assert (yield dut.ready) == 1
        yield ready.eq(0)
        yield
        assert (yield dut.ready) == 0

    run_simulation(dut, check())


def test_memory_and_boot_are_independent(tmp_path):
    binary = tmp_path / "firmware.bin"
    binary.write_bytes(bytes.fromhex("12345678"))
    for boot in ("embedded", "host", "spi"):
        soc     = LiteXModule()
        rams    = []
        masters = []
        soc.add_ram = lambda *args, **kwargs: rams.append((args, kwargs))
        soc.bus = SimpleNamespace(add_master=lambda **kwargs: masters.append(kwargs))
        result = add_wr_cpu_memory(soc, cpu_type="vexriscv", memory="integrated", boot=boot,
            firmware=str(binary), sys_clk_freq=125e6)
        assert rams[0][0] == ("wr_cpu_mem", 0x40000000, 128*1024)
        assert rams[0][1]["contents"] == ([0x78563412] if boot == "embedded" else [])
        assert bool(masters) == (boot == "spi")
        assert (result["cpu_boot_loader"] is not None) == (boot == "spi")


def test_reserved_ddr_region_and_invalid_configuration():
    soc     = LiteXModule()
    region  = SoCRegion(origin=0x51000000, size=128*1024)
    regions = {"main_ram": SoCRegion(origin=0x50000000, size=32*1024*1024)}
    soc.bus = SimpleNamespace(regions=regions,
        add_region=lambda name, region: regions.update({name: region}))
    result = add_wr_cpu_memory(soc, memory="region", boot="host", region=region)
    assert result["cpu_memory_region"] is region
    assert regions["wr_cpu_mem"].origin == region.origin
    assert regions["wr_cpu_mem"].linker
    assert resolve_wr_boot("hyperram") == "spi"
    assert resolve_wr_boot("private") == "embedded"
    for memory, boot in (("private", "host"), ("private", "spi"), ("region", "embedded")):
        with pytest.raises(ValueError):
            add_wr_cpu_memory(LiteXModule(), memory=memory, boot=boot, region=region)
    with pytest.raises(ValueError, match="128 KiB"):
        add_wr_cpu_memory(LiteXModule(), memory="region", boot="host",
            region=SoCRegion(origin=0x50000000, size=64*1024))
