#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""WRPC on a CPU the enclosing SoC owns: the core's wiring, the firmware
profiles named after each CPU, and the Tang Mega 138K Pro's CPU selection."""

import importlib.util
import subprocess

from pathlib import Path
from types import SimpleNamespace

import pytest

from migen import Instance, Memory, Record

from litex_wr_nic.gateware import wr_core
from litex_wr_nic.gateware.wr_core import WhiteRabbitCore
from litex_wr_nic.gateware.wr_cpu  import (
    WR_CORE_CPU_TYPES,
    WR_CPU_PROFILES,
    WR_CPU_TYPES,
    resolve_wr_cpu_variant,
    wr_cpu_firmware_filename,
)

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
    # "external" describes the core's wiring, not a CPU: a target selects it
    # in its own source, together with the peripheral window, interrupt and
    # reset it connects.
    assert "external" in WR_CPU_TYPES
    assert "external" not in WR_CORE_CPU_TYPES
    assert "external" not in WR_CPU_PROFILES
    for name in ("acorn_wr_nic.py", "spec_a7_wr_nic.py", "hyvision_pcie_opt01_revf.py"):
        source = (ROOT / name).read_text(encoding="utf-8")
        assert "choices=WR_CORE_CPU_TYPES" in source, name
    # The firmware profiles are selectable, by the CPU that runs the image.
    for name in ("litex_wr_nic/firmware/build.py", "tang_mega_138k_pro_wr.py"):
        assert "choices=WR_CPU_PROFILES" in (ROOT / name).read_text(encoding="utf-8"), name


def test_a_firmware_profile_is_named_after_its_cpu():
    # The profile follows the CPU, not the core's wiring, so a SoC-owned
    # VexRiscv and a core-instantiated one share one image.
    assert WR_CPU_PROFILES == ("urv", "vexriscv", "ae350")
    assert wr_cpu_firmware_filename("ae350", "bin", "tang_mega_138k_pro") == \
        "tang_mega_138k_pro_wrc_ae350.bin"
    assert wr_cpu_firmware_filename("vexriscv", "bin", "tang_mega_138k_pro") == \
        "tang_mega_138k_pro_wrc_vexriscv.bin"
    # The uRV image keeps the historical name.
    assert wr_cpu_firmware_filename("urv", "bin", "tang_mega_138k_pro") == \
        "tang_mega_138k_pro_wrc.bin"
    with pytest.raises(ValueError, match="firmware profile"):
        wr_cpu_firmware_filename("external", "bin")


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
    (firmware / wr_cpu_firmware_filename("ae350", "bram", "spec_a7")).write_text("00000013\n")
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
    build.configure_cpu_profile("ae350", peripheral_origin=0xe900_0000)
    # WR decodes address bits 15:2, so relocating the window is a new base.
    assert "#define DEV_BASE\t0xe9000000" in board.read_text(encoding="utf-8")


def test_ae350_starts_from_a_bare_reset_with_its_caches_on(firmware):
    build, path = firmware
    build.configure_cpu_profile("ae350")
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
    build.configure_cpu_profile("ae350")
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

# Tang Mega 138K Pro Target Tests ------------------------------------------------------------------

# The first word of each profile's image, so a build shows which one it took.
PROFILE_MARK = {profile: index for index, profile in enumerate(WR_CPU_PROFILES)}


@pytest.fixture
def tang_soc(monkeypatch, tmp_path):
    """Build the Tang target for one WR CPU, from stand-in firmware images."""
    import tang_mega_138k_pro_wr as target

    monkeypatch.setattr(WhiteRabbitCore, "add_sources", staticmethod(lambda platform: None))
    monkeypatch.setattr(target, "FIRMWARE_DIR", tmp_path)
    for profile, mark in PROFILE_MARK.items():
        image = tmp_path / wr_cpu_firmware_filename(profile, "bram", "tang_mega_138k_pro")
        image.write_text("00000013\n", encoding="utf-8")
        image.with_suffix(".bin").write_bytes(bytes([mark, 0, 0, 0])*4)
    return target, lambda wr_cpu_type: target.BaseSoC(wr_cpu_type=wr_cpu_type)


def test_every_cpu_the_board_offers_maps_to_a_core_arrangement(tang_soc):
    target, _ = tang_soc
    assert set(target.WR_CPU_CORE_TYPES) == set(WR_CPU_PROFILES)
    assert set(target.WR_CPU_CORE_TYPES.values()) <= set(WR_CPU_TYPES)
    # The hard CPU belongs to the SoC; the core exposes its window for it.
    assert target.WR_CPU_CORE_TYPES["ae350"] == "external"


def test_the_urv_keeps_its_single_cycle_memory_inside_the_core(tang_soc):
    target, build = tang_soc
    soc = build("urv")
    # Private to the core, and reachable by the host for firmware loading.
    assert soc.wr_core.cpu_bus is None
    assert soc.bus.slaves["wr_cpu_ram"] is soc.wr_core.cpu_memory_bus
    assert soc.bus.regions["wr_cpu_ram"].origin == target.WR_CPU_LOCAL_MEMORY_ORIGIN
    assert "wr_cpu_mem" not in soc.bus.regions
    assert soc.cpu.name == "None"
    banks = [special for special in soc.wr_core.wr_cpu_memory._fragment.specials
        if isinstance(special, Memory)]
    assert len(banks) == 4
    assert banks[0].init[0] == PROFILE_MARK["urv"]


def test_the_vexriscv_runs_from_soc_memory_the_core_masters(tang_soc):
    from litex_wr_nic.gateware.wr_cpu import WR_CPU_MEMORY_ORIGIN

    target, build = tang_soc
    soc = build("vexriscv")
    assert soc.wr_core.cpu_bus is not None
    assert "wr_cpu" in soc.bus.masters
    assert soc.bus.regions["wr_cpu_mem"].origin == WR_CPU_MEMORY_ORIGIN
    assert soc.wr_cpu_mem.mem.init[0] == PROFILE_MARK["vexriscv"]
    assert soc.cpu.name == "None"


def test_the_ae350_boots_wrpc_from_the_cpus_own_memory(tang_soc):
    target, build = tang_soc
    soc = build("ae350")
    regions = soc.bus.regions
    # The reset address is fixed in the hard core; a stub jumps to the image.
    assert regions["rom"].origin == soc.cpu.reset_address
    # lui t0, 0 / jalr x0, 0(t0).
    assert target.AE350_BOOT_STUB == [0x0000_02b7, 0x0002_8067]
    assert soc.rom.mem.init[:2] == target.AE350_BOOT_STUB
    # WRPC keeps its own linker script, so the image lands at 0.
    assert regions["sram"].origin == target.AE350_FIRMWARE_ORIGIN
    assert regions["sram"].size == 128*1024
    assert soc.sram.mem.init[0] == PROFILE_MARK["ae350"]
    # The core owns no memory of its own here.
    assert soc.wr_core.cpu_bus is None
    assert soc.wr_core.cpu_memory_bus is None


def test_the_ae350_maps_wrpc_uncached_and_wires_its_interrupt(tang_soc):
    from migen.fhdl.structure import _Assign, _Slice

    target, build = tang_soc
    soc = build("ae350")
    region = soc.bus.regions["wr_cpu_periph"]
    assert region.origin == target.AE350_PERIPHERAL_ORIGIN
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
