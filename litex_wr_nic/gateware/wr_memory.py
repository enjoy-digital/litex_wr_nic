#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import CSRStorage, CSRStatus
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.common import get_mem_data

from litex_wr_nic.gateware.wr_cpu import (
    WRCPUFlashBoot, WR_CPU_MEMORY_ORIGIN, WR_CPU_MEMORY_SIZE,
    validate_wr_cpu_config, wr_cpu_word_bus,
)

# WR CPU Boot Policy -------------------------------------------------------------------------------

class WRCPUHostBoot(LiteXModule):
    """Hold the CPU until the host has loaded and verified its memory."""
    def __init__(self, memory_ready=1):
        self.ready         = Signal()
        self._host_ready = CSRStorage(
            description="Release the WR CPU after the host has loaded and verified its firmware.")
        self._memory_ready = CSRStatus(
            description="The SoC memory controller is ready for WR CPU accesses.")

        # # #

        self.comb += [
            self.ready.eq(self._host_ready.storage & memory_ready),
            self._memory_ready.status.eq(memory_ready),
        ]


def resolve_wr_boot(memory, boot="auto"):
    if boot == "auto":
        boot = "embedded" if memory in ("private", "integrated") else "spi"
    if boot not in ("embedded", "spi", "host"):
        raise ValueError(f"Unsupported WR CPU boot source: {boot}")
    if memory == "private" and boot != "embedded":
        raise ValueError("Private uRV memory requires embedded boot; use SoC memory for SPI/host boot.")
    if memory not in ("private", "integrated") and boot == "embedded":
        raise ValueError("Embedded boot requires private or integrated memory with FPGA initialization.")
    return boot


def add_wr_cpu_memory(soc, cpu_type="urv", memory="integrated", boot="auto",
    firmware=None, region=None, memory_ready=1, sys_clk_freq=None, allow_legacy_boot=True):
    """Prepare CPU memory/boot independently of the board's memory controller.

    ``memory='region'`` uses a reserved region already backed by the SoC bus,
    including HyperRAM or DDR. The caller owns its controller, cache, physical
    readiness signal and reservation against other software/DMA users.
    Returned keyword arguments can be passed directly to add_white_rabbit.
    """
    if memory not in ("private", "integrated", "region"):
        raise ValueError(f"Unsupported WR memory integration: {memory}")
    validate_wr_cpu_config(cpu_type, memory=memory)
    boot = resolve_wr_boot(memory, boot)
    if boot == "spi" and sys_clk_freq is None:
        raise ValueError("SPI boot requires the system clock frequency.")
    loader = None
    ready  = memory_ready
    if memory == "private":
        if region is not None:
            raise ValueError("Private memory cannot use a SoC memory region.")
        return dict(cpu_memory_region=None, cpu_memory_ready=1, cpu_boot_loader=None)
    if region is None:
        if memory == "region":
            raise ValueError("A reserved, bus-backed SoC region is required.")
        region = SoCRegion(origin=WR_CPU_MEMORY_ORIGIN, size=WR_CPU_MEMORY_SIZE, mode="rwx")
    if region.size != WR_CPU_MEMORY_SIZE or region.origin % WR_CPU_MEMORY_SIZE:
        raise ValueError("WR memory requires a 128 KiB region aligned to 128 KiB.")
    if "w" not in region.mode or "r" not in region.mode:
        raise ValueError("WR CPU memory must be readable and writable.")
    if memory == "region":
        regions = soc.bus.regions
        if not any(
            region.origin >= parent.origin and
            region.origin + region.size <= parent.origin + parent.size and
            not parent.linker
            for parent in regions.values()
        ):
            raise ValueError("The reserved WR memory region must be backed by an existing SoC region.")
        existing = regions.get("wr_cpu_mem")
        if existing is not None:
            if existing.origin != region.origin or existing.size != region.size:
                raise ValueError("wr_cpu_mem already refers to a different memory region.")
        else:
            # A named subregion provides host discovery without adding a second
            # slave decoder over the DDR controller. Software must reserve it.
            soc.bus.add_region("wr_cpu_mem", SoCRegion(
                origin = region.origin,
                size   = region.size,
                mode   = region.mode,
                cached = region.cached,
                linker = True,
            ))
    if memory == "integrated":
        contents = []
        if boot == "embedded":
            if firmware is None:
                raise ValueError("Embedded WR boot requires a firmware binary.")
            contents = get_mem_data(firmware,
                data_width = 32,
                endianness = "little",
                mem_size   = region.size,
            )
        soc.add_ram("wr_cpu_mem", region.origin, region.size, contents=contents)
    if boot == "spi":
        soc.wr_cpu_boot = loader = WRCPUFlashBoot(sys_clk_freq,
            cpu_type     = cpu_type,
            memory_ready = memory_ready,
            allow_legacy = allow_legacy_boot,
        )
        soc.bus.add_master(name="wr_cpu_boot", master=wr_cpu_word_bus(soc, loader.bus), region=region)
        ready = loader.ready
    elif boot == "host":
        soc.wr_cpu_boot = host = WRCPUHostBoot(memory_ready)
        ready = host.ready
    return dict(cpu_memory_region=region, cpu_memory_ready=ready, cpu_boot_loader=loader)
