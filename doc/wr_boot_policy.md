# Choosing WR memory and boot source

On SPEC-A7, `--wr-cpu-memory` selects storage and `--wr-cpu-boot` selects how
firmware reaches it. Defaults remain uRV, private memory and embedded firmware.

| Memory | Embedded | SPI | Host |
| --- | --- | --- | --- |
| Private uRV RAM | Default | Use SoC memory | Use SoC memory |
| Integrated SoC RAM | Default | Supported | Supported |
| HyperRAM | No FPGA initialization | Default | Supported |
| Reserved DDR/other SoC region | Controller dependent; use SPI/host | Generic helper | Generic helper |

For example, build an image that waits for a host-supplied VexRiscv firmware:

```sh
python3 spec_a7_wr_nic.py --build --wr-cpu-type vexriscv \
    --wr-cpu-memory integrated --wr-cpu-boot host --skip-firmware-build
```

After loading that bitstream and starting `litex_server`, upload firmware:

```sh
python3 -m litex_wr_nic.wr --csr-csv test/csr.csv load-firmware \
    litex_wr_nic/firmware/spec_a7_wrc_vexriscv.boot
```

Use `--wr-cpu-memory hyperram --wr-cpu-boot host` for the same flow through
the HyperRAM controller and shared cache. Host boot holds the CPU until the
memory controller is ready and the host has written and verified all 128 KiB.
Failed validation leaves the CPU held. A host reboot or disconnect does not
release a waiting CPU. A normal CPU restart keeps the verified memory image.

SPEC-A7 samples HyperRAM DQ/RWDS with a 180-degree shifted system PLL output.
The cache uses local 17-bit byte addresses for the reserved 128 KiB window;
the SoC decoder enforces that window's address bounds. Full host readback
exercises physical RAM after cache eviction, which is essential: immediate
readback from the cache alone did not expose the read corruption observed
with the original sampling configuration. The system clock remains derived
from its PLL so implementation checks its real relationship to the input
capture clock.

`--wr-cpu-memory integrated --wr-cpu-boot spi` reads the same SPI boot slot as
HyperRAM boot. The default `auto` boot source preserves the previous selection
for each memory mode. Private uRV RAM is physically inside the WR wrapper;
select SoC-backed memory for interchangeable loaders.

## Reusing a memory region

`add_wr_cpu_memory` prepares a SoC memory window and boot controller. Pass its
return value as keyword arguments to `add_white_rabbit`/`add_wr_core`:

```python
from litex.soc.integration.soc import SoCRegion
from litex_wr_nic.gateware.wr_memory import add_wr_cpu_memory

# This aligned 128 KiB window must be inside an existing DDR bus slave.
# Reserve it from the main CPU's allocator/linker and every DMA user.
wr_memory = add_wr_cpu_memory(self,
    cpu_type     = "vexriscv",
    memory       = "region",
    region       = SoCRegion(origin=wr_reserved_origin, size=128*1024),
    memory_ready = ddr_initialized,
    boot         = "host",
)
self.add_wr_core(cpu_type="vexriscv", cpu_firmware="unused.bram",
    sfp_pads=sfp_pads, sfp_i2c_pads=sfp_i2c_pads, **wr_memory)
```

The helper checks the backing region and exports a `wr_cpu_mem` subregion for
host discovery. It does not allocate or enforce the software reservation.
For `memory="integrated"` it creates the RAM; embedded boot additionally
requires `firmware="path/to/profile.bin"`. SPI boot requires `sys_clk_freq`
and the core's flash pads/loader connection. The caller owns controller and
cache initialization. Both CPU and host must use the same cache path; bypass
writes are not coherent with cached CPU reads. The current VexRiscv adapter's
instruction cache is reset with its CPU during reload.

## Boot package metadata

The firmware builder now emits version 2 `.boot` files. The 32-byte
little-endian header contains magic, version, payload length, payload CRC32,
CPU ID (uRV=0, VexRiscv=1), WRPC ABI ID (1), load address (0) and entry point
(0). A loader rejects a CPU/ABI/address mismatch before issuing memory writes.
Length and CRC checks still gate CPU release. Images contain a zero-filled
128 KiB memory window.

The host tool reads the profile from this header. Raw binaries and legacy
version 1 packages still require `--firmware-cpu`. FPGA loaders accept legacy
version 1 by default for existing flash contents; these images have no CPU
metadata. Integrations can set `allow_legacy_boot=False` to require version 2.
The standalone packager accepts `--cpu-type urv|vexriscv`; omitting it preserves
the legacy format for older bitstreams.

Validation covers both CPU profiles, metadata rejection before writes,
memory-controller readiness, host verification and the supported SPEC-A7
elaborations. DDR behavior needs qualification with the selected controller
and application; SPEC-A7 provides HyperRAM rather than DDR.
