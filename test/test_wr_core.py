#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import sys
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

from migen import Instance, Record, Signal, run_simulation

from litex.gen import LiteXModule

from litex.soc.interconnect.csr_bus import CSRBankArray
from litex.soc.integration.soc import SoCRegion

from litex_wr_nic.gateware import wr_core
from litex_wr_nic.gateware.wr_core import WhiteRabbitCore, add_white_rabbit

# Helpers ------------------------------------------------------------------------------------------

def core_kwargs():
    return dict(
        cpu_firmware = "unused.bram",
        sfp_pads     = Record([(name, 1) for name in ("txp", "txn", "rxp", "rxn")]),
        sfp_i2c_pads = Record([("sda", 1), ("scl", 1)]),
    )


@pytest.fixture
def platform(monkeypatch):
    sources  = []
    prepared = []
    for name in (
        "wr_core_init", "patch_wr_subsystem_mux_class", "patch_wr_pps_gen_iob",
        "patch_wr_clock_monitor_presc_cdc", "patch_wr_external_cpu_memory",
    ):
        monkeypatch.setattr(wr_core, name, lambda name=name: prepared.append(name))
    return SimpleNamespace(
        device     = "xc7a50t",
        add_source = sources.append,
        sources    = sources,
        prepared   = prepared,
    )

# Reusable Core Tests ------------------------------------------------------------------------------

def test_core_import_has_no_nic_or_soc_side_effects():
    # Use a fresh interpreter: other tests import the compatibility NIC class.
    script = """
import sys
from litex.soc.integration.soc_core import SoCMini
before = (dict(SoCMini.csr_map), dict(SoCMini.mem_map))
from litex_wr_nic.gateware.wr_core import WhiteRabbitCore
assert 'liteeth.mac.sram' not in sys.modules
assert 'litepcie.frontend.ptm' not in sys.modules
from litex_wr_nic.gateware.soc import LiteXWRNICSoC
assert before == (SoCMini.csr_map, SoCMini.mem_map)
"""
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path))
    subprocess.run([sys.executable, "-c", script], env=env, check=True)


def test_standalone_core_interfaces(platform):
    core = WhiteRabbitCore(platform, **core_kwargs())
    assert core.bus.addressing == "word"
    assert core.cpu_bus is None
    assert core.sink is core.wrf_stream2wb.sink
    assert core.source is core.wrf_wb2stream.source
    fragment = core.get_fragment()
    instances = [s for s in fragment.specials if isinstance(s, Instance)]
    assert len(instances) == 1
    uart = next(item.expr for item in instances[0].items if item.name == "uart_rxd_i")
    assert uart.value == 1


@pytest.mark.parametrize("enabled", [False, True])
def test_external_clock_generic_is_numeric(platform, enabled):
    core = WhiteRabbitCore(platform, with_ext_clk=enabled, **core_kwargs())
    instance = next(s for s in core.get_fragment().specials
        if isinstance(s, Instance) and s.of == "xwrc_board_litex_wr_nic_wrapper")
    parameter = next(item.value for item in instance.items
        if isinstance(item, Instance.Parameter) and item.name == "g_with_external_clock_input")
    # Quoted FALSE can bind as true at the Verilog/VHDL boundary in Vivado.
    assert not isinstance(parameter, str)
    assert parameter.value == int(enabled)


def test_compatibility_adapter_registers_memory_and_csrs_once(platform):
    soc          = LiteXModule()
    soc.platform = platform
    masters      = {}
    slaves       = {}
    soc.bus = SimpleNamespace(
        add_master = lambda **kwargs: masters.update(kwargs),
        add_slave  = lambda **kwargs: slaves.update(kwargs),
    )
    region = SoCRegion(origin=0x50000000, size=128*1024)
    core = add_white_rabbit(soc, cpu_memory_region=region, **core_kwargs())
    assert masters["region"] is region
    assert masters["master"] is core.cpu_bus
    assert slaves["slave"] is core.bus
    assert slaves["region"].origin == 0x20000000
    assert slaves["region"].size == 0x01000000
    banks = CSRBankArray(soc, lambda name, memory: 5)
    assert [name for name, *_ in banks.banks] == ["wr_cpu_bridge"]
    assert [csr.name for csr in banks.banks[0][1]] == [
        "status", "error_count", "last_error_address",
    ]
    fragment = soc.get_fragment()
    assert sum(isinstance(s, Instance) and s.of == "xwrc_board_litex_wr_nic_wrapper"
        for s in fragment.specials) == 1


@pytest.mark.parametrize("explicit", [False, True])
def test_sources_are_automatic_and_explicit_calls_are_idempotent(platform, monkeypatch, tmp_path, explicit):
    monkeypatch.chdir(tmp_path)
    core = WhiteRabbitCore(platform, **core_kwargs())
    assert platform.sources == []
    if explicit:
        WhiteRabbitCore.add_sources(platform)
    core.get_fragment()
    assert platform.sources == list(wr_core.wr_core_files)
    assert len(platform.prepared) == 5
    assert platform.prepared[0] == "wr_core_init"
    WhiteRabbitCore.add_sources(platform)
    assert platform.sources == list(wr_core.wr_core_files)
    assert len(platform.prepared) == 5


@pytest.mark.parametrize("size", [0x100000, 0x60000])
def test_host_region_preserves_attributes_and_decodes_local_addresses(platform, size):
    soc          = LiteXModule()
    soc.platform = platform
    slaves       = {}
    soc.bus      = SimpleNamespace(add_slave=lambda **kwargs: slaves.update(kwargs))
    region = SoCRegion(origin=0x20100000, size=size, mode="rw", cached=False)
    core = add_white_rabbit(soc, wb_slave_region=region, **core_kwargs())
    assert slaves["region"] is region
    assert slaves["slave"] is core.bus

    # Check the address actually presented to WR, including a base that is
    # aligned to the decoded region but not to four times that region.
    fragment = core.get_fragment()
    instance = next(s for s in fragment.specials
        if isinstance(s, Instance) and s.of == "xwrc_board_litex_wr_nic_wrapper")
    address = next(item.expr for item in instance.items if item.name == "wb_slave_adr")
    dut = LiteXModule()
    local_address = Signal(32)
    dut.comb += local_address.eq(address)

    def check():
        for offset in (0, 4, 0x20b00, region.size_pow2 - 4):
            yield core.wb_slave_wr.adr.eq((region.origin + offset) // 4)
            yield
            assert (yield local_address) == offset

    run_simulation(dut, check())


@pytest.mark.parametrize("cpu_type, section", [
    ("urv", "uRV with private RAM"),
    ("vexriscv", "VexRiscv with integrated RAM"),
])
def test_documented_cpu_setups_elaborate(platform, monkeypatch, tmp_path, cpu_type, section):
    from litex.soc.integration.soc_core import SoCMini
    from spec_a7_platform import Platform

    documentation = Path(__file__).resolve().parents[1] / "doc/wr_integration.md"
    example = documentation.read_text(encoding="utf-8").split("## " + section + "\n", 1)[1]
    code    = example.split("```python\n", 1)[1].split("```", 1)[0]
    monkeypatch.chdir(tmp_path)
    firmware = tmp_path / "litex_wr_nic/firmware"
    firmware.mkdir(parents=True)
    stem = "spec_a7_wrc" + ("_vexriscv" if cpu_type == "vexriscv" else "")
    (firmware / (stem + ".bram")).write_text("00000013\n", encoding="utf-8")
    (firmware / (stem + ".bin")).write_bytes(bytes.fromhex("13000000"))
    soc = SoCMini(Platform(), clk_freq=125e6, ident_version=False)
    exec(compile(code, str(documentation), "exec"), {"self": soc})
    soc.finalize()
    assert soc.bus.regions["wr_wb_slave"].cached is False
    assert soc.platform._wr_core_sources_added
    if cpu_type == "vexriscv":
        assert soc.wr_core.cpu_bus is not None
        assert soc.bus.regions["wr_cpu_mem"].origin == 0x50000000
        assert "wr_cpu" in soc.bus.masters
    else:
        assert soc.wr_core.cpu_bus is None
        assert "wr_cpu" not in soc.bus.masters
