#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""WRPC driven by a CPU the enclosing SoC owns, and the Gowin AE350 profile."""

import importlib.util
import subprocess

from pathlib import Path
from types import SimpleNamespace

import pytest

from migen import Instance, Record

from litex_wr_nic.gateware import wr_core
from litex_wr_nic.gateware.wr_core import WhiteRabbitCore
from litex_wr_nic.gateware.wr_cpu  import resolve_wr_cpu_variant, wr_cpu_firmware_filename

# Constants ----------------------------------------------------------------------------------------

ROOT   = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "litex_wr_nic/firmware/wrpc-sw"

# Helpers ------------------------------------------------------------------------------------------

def core_kwargs():
    return dict(
        cpu_firmware = "unused.bram",
        sfp_pads     = Record([(name, 1) for name in ("txp", "txn", "rxp", "rxn")]),
        sfp_i2c_pads = Record([("sda", 1), ("scl", 1)]),
    )


@pytest.fixture
def platform(monkeypatch):
    sources = []
    for name in (
        "wr_core_init", "patch_wr_subsystem_mux_class", "patch_wr_pps_gen_iob",
        "patch_wr_clock_monitor_presc_cdc", "patch_wr_external_cpu_memory",
        "patch_wr_diags_control_word", "patch_wr_gtx_clocking",
    ):
        monkeypatch.setattr(wr_core, name, lambda: None)
    return SimpleNamespace(device="xc7a50t", add_source=sources.append, sources=sources)


def core_instance(core):
    return next(special for special in core.get_fragment().specials
        if isinstance(special, Instance) and special.of == "xwrc_board_litex_wr_nic_wrapper")


@pytest.fixture
def firmware(tmp_path, monkeypatch):
    """The pinned WRPC files the AE350 profile rewrites, in a scratch tree."""
    if not SOURCE.exists():
        pytest.skip("Initialize pinned WRPC sources first")
    spec  = importlib.util.spec_from_file_location("wr_build", ROOT / "litex_wr_nic/firmware/build.py")
    build = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(build)
    for name in ("Makefile", "arch/risc-v/crt0.S", "arch/risc-v/irq_helper.c",
                 "include/board.h", "include/irq.h"):
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(subprocess.check_output(
            ["git", "-C", str(SOURCE), "show", build.COMMIT_HASH + ":" + name]))
    monkeypatch.setattr(build, "CLONE_DIR", str(tmp_path))
    return build, tmp_path

# SoC CPU Core Tests -------------------------------------------------------------------------------

def test_soc_cpu_exports_a_peripheral_window_an_irq_and_a_reset(platform):
    core = WhiteRabbitCore(platform, cpu_type="external", **core_kwargs())
    # The SoC's CPU owns its memory: the core neither masters nor hosts one.
    assert core.cpu_bus is None
    assert core.cpu_memory_bus is None
    assert not hasattr(core, "wr_cpu")
    bus = core.cpu_peripheral_bus
    assert (bus.data_width, bus.addressing) == (32, "word")

    instance   = core_instance(core)
    parameters = {item.name: item.value for item in instance.items
        if isinstance(item, Instance.Parameter)}
    # WRPC's own CPU and its private memory are both left out; the unused
    # memory port stays enabled so the upstream generate's assertion holds.
    assert parameters["g_external_cpu"].value == 1
    assert parameters["g_external_cpu_memory"].value == 1
    outputs = {item.name: item.expr for item in instance.items
        if isinstance(item, Instance.Output)}
    assert outputs["cpu_ext_irq_o"] is core.cpu_irq
    assert outputs["cpu_ext_reset_o"] is core.cpu_reset


def test_soc_cpu_does_not_take_the_cores_private_memory(platform):
    assert resolve_wr_cpu_variant("external") is None
    with pytest.raises(ValueError, match="owns its memory"):
        WhiteRabbitCore(platform, cpu_type="external", with_cpu_memory=True, **core_kwargs())


def test_targets_only_offer_the_cpus_they_wire():
    from litex_wr_nic.gateware.wr_cpu import WR_CORE_CPU_TYPES, WR_CPU_TYPES

    # A target selects "external" in its own source, together with the
    # peripheral window, interrupt and reset it wires to its CPU.
    assert "external" in WR_CPU_TYPES
    assert "external" not in WR_CORE_CPU_TYPES
    for name in ("acorn_wr_nic.py", "spec_a7_wr_nic.py", "hyvision_pcie_opt01_revf.py"):
        source = (ROOT / name).read_text(encoding="utf-8")
        assert "choices=WR_CORE_CPU_TYPES" in source, name
    # The firmware profile is selectable, since a target builds it by name.
    assert "choices=WR_CPU_TYPES" in (ROOT / "litex_wr_nic/firmware/build.py").read_text(encoding="utf-8")


def test_soc_cpu_firmware_is_a_profile_of_its_own():
    # The image differs from the uRV one by peripheral window and CPU quirks.
    assert wr_cpu_firmware_filename("external", "bin", "tang_mega_138k_pro") == \
        "tang_mega_138k_pro_wrc_ae350.bin"
    assert wr_cpu_firmware_filename("urv", "bin", "tang_mega_138k_pro") == \
        "tang_mega_138k_pro_wrc.bin"


def test_documented_soc_cpu_setup_elaborates(monkeypatch, tmp_path):
    from migen import Signal
    from litex.soc.integration.soc_core import SoCMini
    from spec_a7_platform import Platform

    documentation = ROOT / "doc/wr_integration.md"
    example = documentation.read_text(encoding="utf-8").split("## A CPU the SoC owns\n", 1)[1]
    code    = example.split("```python\n", 1)[1].split("```", 1)[0]
    monkeypatch.chdir(tmp_path)
    firmware = tmp_path / "litex_wr_nic/firmware"
    firmware.mkdir(parents=True)
    (firmware / wr_cpu_firmware_filename("external", "bram", "spec_a7")).write_text("00000013\n")
    soc = SoCMini(Platform(), clk_freq=125e6, ident_version=False)
    # Stand in for the SoC's own CPU and its interrupt inputs.
    soc.cpu.interrupt = Signal(16)
    exec(compile(code, str(documentation), "exec"), {"self": soc})
    soc.finalize()
    assert soc.wr_core.cpu_bus is None
    assert soc.bus.regions["wr_cpu_periph"].cached is False
    assert "wr_cpu" not in soc.bus.masters

# AE350 Firmware Profile Tests ---------------------------------------------------------------------

def test_peripheral_window_follows_the_soc_address_decoder(firmware):
    build, path = firmware
    board = path / "include/board.h"
    assert "#define DEV_BASE\t0x100000" in board.read_text(encoding="utf-8")
    build.configure_cpu_profile("external", peripheral_origin=0xe900_0000)
    # WR decodes address bits 15:2, so relocating the window is a new base.
    assert "#define DEV_BASE\t0xe9000000" in board.read_text(encoding="utf-8")


def test_ae350_starts_from_a_bare_reset_with_its_caches_on(firmware):
    build, path = firmware
    build.configure_cpu_profile("external")
    crt0 = (path / "arch/risc-v/crt0.S").read_text(encoding="utf-8")
    entry = crt0.split("_entry:", 1)[1].split("la     gp, _gp", 1)[0]
    # The hard core resets with mtvec elsewhere and both caches disabled.
    assert "csrw   mtvec, t0" in entry
    assert "li     t0, 0x3" in entry and "csrs   0x7ca, t0" in entry
    # The uRV builds must not pick any of this up.
    assert "#if defined(WR_CPU_VEXRISCV) || defined(WR_CPU_AE350)" in entry
    assert "#ifdef WR_CPU_AE350" in entry


def test_the_wr_interrupt_is_claimed_and_completed_at_the_plic(firmware):
    build, path = firmware
    build.configure_cpu_profile("external")
    header = (path / "include/irq.h").read_text(encoding="utf-8")
    profile = header.split("#elif defined(CONFIG_ARCH_RISCV) && defined(WR_CPU_AE350)", 1)[1]
    profile = profile.split("#elif defined(CONFIG_ARCH_RISCV)", 1)[0]
    assert "#define AE350_PLIC_BASE      0xe4000000" in profile
    # A claim that is never completed leaves the external interrupt asserted.
    assert "AE350_PLIC_CLAIM  = source;" in profile
    assert "AE350_PLIC_THRESHOLD = 0;" in profile
    # The generic RISC-V path is preserved for the other CPUs.
    assert 'asm volatile ("csrrc %0, mip, %1"' in header
    assert "ae350_plic_source" in (path / "arch/risc-v/irq_helper.c").read_text(encoding="utf-8")

# AE350 Target Tests -------------------------------------------------------------------------------

@pytest.fixture
def ae350_soc(monkeypatch, tmp_path):
    import tang_mega_138k_pro_wr_ae350 as target

    monkeypatch.setattr(WhiteRabbitCore, "add_sources", staticmethod(lambda platform: None))
    firmware = tmp_path / wr_cpu_firmware_filename("external", "bin", "tang_mega_138k_pro")
    firmware.write_bytes(bytes(range(16)))
    return target, target.BaseSoC(cpu_firmware=str(firmware))


def test_ae350_target_boots_wrpc_from_the_cpus_own_memory(ae350_soc):
    target, soc = ae350_soc
    regions = soc.bus.regions
    # The reset address is fixed in the hard core; a stub jumps to the image.
    assert regions["rom"].origin == soc.cpu.reset_address
    # lui t0, 0 / jalr x0, 0(t0).
    assert target.BOOT_STUB == [0x0000_02b7, 0x0002_8067]
    assert soc.rom.mem.init[:2] == target.BOOT_STUB
    # WRPC keeps its own linker script, so the image lands at 0.
    assert regions["sram"].origin == 0x0000_0000
    assert regions["sram"].size == 128*1024
    assert soc.sram.mem.init[:4] == [0x03020100, 0x07060504, 0x0b0a0908, 0x0f0e0d0c]


def test_ae350_target_maps_wrpc_uncached_and_wires_its_interrupt(ae350_soc):
    from migen.fhdl.structure import _Assign, _Slice

    target, soc = ae350_soc
    region = soc.bus.regions["wr_cpu_periph"]
    assert region.origin == target.WR_PERIPHERAL_ORIGIN
    # WRPC's registers must not be held in the A25's write-back cache.
    assert region.cached is False
    assert soc.bus.slaves["wr_cpu_periph"] is soc.wr_core.cpu_peripheral_bus

    # The SoftPLL interrupt reaches the CPU through its own PLIC source.
    assert not soc.irq.locs, "GP_INT[0] is reserved for the WR SoftPLL."
    assert any(isinstance(statement, _Assign)
        and statement.r is soc.wr_core.cpu_irq
        and isinstance(statement.l, _Slice) and statement.l.value is soc.cpu.interrupt
        and (statement.l.start, statement.l.stop) == (0, 1)
        for statement in soc._fragment.comb)
