# Integrating White Rabbit in a LiteX design

`WhiteRabbitCore` is a `LiteXModule` that can be used without the PCIe NIC,
LiteEth SRAM replacement, or the NIC target's address map. It currently wraps
the Artix-7 GTP and Kintex-7 GTX WR board implementations. See the
[board matrix](boards.md) for target clocking and qualification scope. HDL sources are added
automatically by `do_finalize()`; no separate `add_sources()` call is needed.

The examples below belong inside an existing SoC constructor, after its
platform, bus and board clocking have been created. Choose one CPU setup.
They use SPEC-A7 resource names and firmware; adapt pads and firmware target
for another board. Your SoC supplies the host bus master (for example a LiteX
CPU or JTAGBone).

## CPU, memory and boot architecture

![White Rabbit CPU and memory choices, with the SPEC-A7 flash-to-HyperRAM boot sequence](wr_cpu_architecture.svg)

Select one WR CPU and one compatible memory path at gateware build time. The blocks show
functional interfaces: uRV is embedded in the upstream WR core; VexRiscv is instantiated
through the LiteX CPU adapter. Both run WRPC firmware, including the SoftPLL control loop,
using their respective firmware profiles. The WR endpoint, hardware timestamps, DDMTD phase
measurement and board PHY provide the same interfaces to the selected CPU and application.

| Memory selection | WR CPU | Implementation | Firmware initialization |
| --- | --- | --- | --- |
| `private` (default) | uRV | 128 KiB private WR BRAM | `.bram` contents included in the bitstream |
| `integrated` | uRV or VexRiscv `lite` | 128 KiB LiteX SoC BRAM | Matching CPU `.bin` contents included in the bitstream |
| `hyperram` | uRV or VexRiscv `lite` | SPEC-A7 HyperRAM behind a 16 KiB write-back cache | Matching CPU `.boot` image copied from SPI flash |
| none (SoC-owned) | `external` | A CPU the SoC already has, with its own memory | The CPU's own profile `.bin`, in the SoC's memory |

The CPU sees its 128 KiB firmware RAM at address zero. For the SoC-memory paths, instruction
and low-memory data accesses cross from `wr_sys` to `sys`; the supplied NIC targets map
`wr_cpu_mem` at `0x40000000`. A custom SoC can reserve another region, as in the integrated-RAM
example below. CPU accesses to WR peripherals remain on the WR peripheral bus. Host control,
application Ethernet streams and time/PPS interfaces remain available independently of the
CPU/memory selection. JTAGBone, UARTBone and PCIe access depend on the target configuration.

On SPEC-A7, `--wr-cpu-memory hyperram` instantiates the FPGA flash loader. It holds the WR CPU
in reset while memory starts up, reads the selected CPU's boot image at flash offset
`0x002f0000`, checks the header, and copies the payload through the SoC bus/cache to HyperRAM.
After a successful CRC32 check, it releases the CPU to execute from RAM and returns flash
ownership to WRPC for SDB access. A header, CRC or bus error keeps the CPU in reset; loader
status is available through the `wr_cpu_boot` CSRs. See
[CPU and memory selection](../README.md#-select-the-wr-cpu) for build commands and flash layout.

`external` is the third choice: the SoC keeps its own CPU, its own memory and its own
interrupt controller, and the core only exports what WRPC needs from it. The Tang Mega
138K Pro AE350 target uses it for the device's hardened Andes A25.

The current LiteX CPU adapter supports VexRiscv `lite`; additional LiteX CPUs need an adapter
and matching firmware support. Integrated RAM is available on SPEC-A7, Acorn and HyVision;
the supplied HyperRAM/flash-boot implementation is specific to SPEC-A7. Consult the
[board matrix](boards.md) for hardware qualification status.

The [diagram source](wr_cpu_architecture.svg) is a self-contained SVG with editable text and
named groups. Edit it directly in Inkscape or another SVG editor; the README and this guide
use the same file. To export it from the repository root for a presentation:

```sh
inkscape doc/wr_cpu_architecture.svg --export-type=png --export-width=1600 \
    --export-filename=/tmp/wr_cpu_architecture.png
```

## Board requirements

The board must supply `sys`, `clk_62m5_dmtd`, `clk_125m_gtp` and `clk10m_in`.
The WR wrapper produces `wr_sys` and `wr`, including their resets. Connect its
oscillator tuning outputs to the board's DAC/MMCM implementation, and wire
PPS and the optional UART/flash/one-wire interfaces as required. The examples
show SFP/I2C and direct UART connections; they do not replace the board CRG or
oscillator wiring. When sharing transceiver PLL resources, also pass the
board's `qpll`, as in `spec_a7_wr_nic.py`.

## uRV with private RAM

Build the matching embedded firmware from the repository root:

```sh
python3 litex_wr_nic/firmware/build.py --target spec_a7 --wr-cpu-type urv
```

Attach the core and register its host slave in one call:

```python
from litex.soc.integration.soc import SoCRegion
from litex_wr_nic.gateware.wr_core import add_white_rabbit

platform = self.platform
wr = add_white_rabbit(self,
    cpu_type        = "urv",
    cpu_firmware    = "litex_wr_nic/firmware/spec_a7_wrc.bram",
    wb_slave_region = SoCRegion(origin=0x20000000, size=0x01000000, cached=False),
    sfp_pads        = platform.request("sfp", 0),
    sfp_i2c_pads    = platform.request("sfp_i2c", 0),
    serial_pads     = platform.request("serial"),
)
```

Private RAM belongs to the WR wrapper and is initialized from the `.bram`
file. There is no CPU memory master to register and no external memory
readiness signal to supply. The default CPU/memory choice remains uRV/private.

## VexRiscv with integrated RAM

Build the VexRiscv firmware first:

```sh
python3 litex_wr_nic/firmware/build.py --target spec_a7 --wr-cpu-type vexriscv
```

Allocate and initialize the 128 KiB SoC RAM before attaching WR:

```python
from litex.soc.integration.common import get_mem_data
from litex.soc.integration.soc import SoCRegion
from litex_wr_nic.gateware.wr_core import add_white_rabbit

platform = self.platform
contents = get_mem_data("litex_wr_nic/firmware/spec_a7_wrc_vexriscv.bin",
    data_width = 32,
    endianness = "little",
    mem_size   = 0x20000,
)
self.add_ram("wr_cpu_mem", origin=0x50000000, size=0x20000, contents=contents)
wr = add_white_rabbit(self,
    cpu_type          = "vexriscv",
    cpu_variant       = "lite",
    cpu_firmware      = "litex_wr_nic/firmware/spec_a7_wrc_vexriscv.bram",
    cpu_memory_region = self.bus.regions["wr_cpu_mem"],
    cpu_memory_ready  = 1,
    wb_slave_region   = SoCRegion(origin=0x20000000, size=0x01000000, cached=False),
    sfp_pads          = platform.request("sfp", 0),
    sfp_i2c_pads      = platform.request("sfp_i2c", 0),
    serial_pads       = platform.request("serial"),
)
```

The helper registers both the host slave and `wr.cpu_bus`, mapping the CPU's
local memory addresses into `wr_cpu_mem`. `cpu_memory_ready=1` is appropriate
here because integrated RAM contains firmware at FPGA configuration. With an
external memory controller, keep readiness low until initialization completes
and firmware is present. VexRiscv requires SoC memory and its matching firmware;
the current adapter supports the `lite` variant.

All three helper examples create `self.wr_core` and retain the historical
attributes and CSR names. `wr` is a local reference to that module; do not
register it again as another SoC submodule. `LiteXWRNICSoC.add_wr_core(...)`
delegates to the same helper. `wb_slave_region` preserves the supplied region's
attributes and takes precedence over the legacy `wb_slave_origin` and
`wb_slave_size` arguments, which remain supported. LiteX rounds the decoded
region to a power of two; the core uses that same size for local addressing.

## A CPU the SoC owns

Use `cpu_type="external"` when the SoC already has a CPU that should run
WRPC: a hard core, or a LiteX CPU with a memory and boot arrangement of its
own. The core then instantiates no CPU and no memory. It exports three
things, and the SoC connects all three.

Build the matching firmware first. `--wr-cpu-type` names the CPU that runs
the image, and `--peripheral-origin` the address the SoC decodes WRPC's
peripheral window at:

```sh
python3 litex_wr_nic/firmware/build.py --target spec_a7 --wr-cpu-type ae350 \
    --peripheral-origin 0xe9000000
```

```python
from litex.soc.integration.soc import SoCRegion
from litex_wr_nic.gateware.wr_core import add_white_rabbit

platform = self.platform
wr = add_white_rabbit(self,
    cpu_type        = "external",
    cpu_firmware    = "litex_wr_nic/firmware/spec_a7_wrc_ae350.bram",
    wb_slave_region = SoCRegion(origin=0x20000000, size=0x01000000, cached=False),
    sfp_pads        = platform.request("sfp", 0),
    sfp_i2c_pads    = platform.request("sfp_i2c", 0),
    serial_pads     = platform.request("serial"),
)
# WRPC's peripherals, at the address the firmware was built for. WR decodes
# address bits 15:2 only, so the window can go anywhere the CPU leaves
# uncached.
self.bus.add_slave(name="wr_cpu_periph", slave=wr.cpu_peripheral_bus,
    region=SoCRegion(origin=0xe9000000, size=0x10000, cached=False))
# The SoftPLL interrupt. Without it the servo runs from polling alone and
# will not hold phase.
self.comb += self.cpu.interrupt[0].eq(wr.cpu_irq)
```

`wr.cpu_peripheral_bus` is a 32-bit word-addressed classic Wishbone slave; the
core's bridge turns each access into a pipelined WR transaction. `wr.cpu_irq`
is the SoftPLL interrupt and `wr.cpu_reset` is WRPC's own reset request, which
the SoC may route to its CPU's reset.

The firmware must load its own image, set `mtvec` and service the interrupt
the way the CPU's platform requires, so the profile follows the CPU and not
this wiring: `--wr-cpu-type ae350` relocates `DEV_BASE`, initializes `mtvec`,
enables the A25's caches and claims and completes each interrupt at that
CPU's PLIC. A SoC-owned VexRiscv uses the same `vexriscv` profile as a
core-instantiated one, with its own `--peripheral-origin`; another CPU needs a
profile of its own. `cpu_firmware` still names a `.bram` file; the core's
private memory is left out, so its contents are unused.

## Direct component integration

To choose a different hierarchy or CSR layout, instantiate the component
and register its interfaces yourself:

```python
from litex.soc.integration.soc import SoCRegion
from litex_wr_nic.gateware.wr_core import WhiteRabbitCore

self.wr = WhiteRabbitCore(self.platform,
    cpu_firmware = "litex_wr_nic/firmware/spec_a7_wrc.bram",
    sfp_pads     = self.platform.request("sfp", 0),
    sfp_i2c_pads = self.platform.request("sfp_i2c", 0),
    serial_pads  = self.platform.request("serial"),
)
self.bus.add_slave("wr", self.wr.bus,
    SoCRegion(origin=0x20000000, size=0x01000000, cached=False))
```

Automatic source registration applies to this form too. Historical explicit
`WhiteRabbitCore.add_sources(platform)` and `self.add_sources()` calls remain
supported and register the WR sources only once per platform. Use Migen's
`ClockDomainsRenamer` when a design needs different clock-domain names.

| Interface | Clock | Contract |
| --- | --- | --- |
| `bus` | `sys` | 32-bit word-addressed host Wishbone slave |
| `cpu_bus` | `sys` | Optional 32-bit word-addressed CPU memory master |
| `cpu_memory_bus` | `sys` | Optional host slave of the local single-cycle CPU memory |
| `sink` / `source` | `sys` | Application Ethernet byte streams |
| `dac_*_data`, `dac_*_load` | `wr_sys` | Oscillator tuning commands |
| `tm_seconds`, `tm_cycles`, `tm_time_valid`, `pps_*` | `wr` | PHY reference time and PPS |

For direct SoC-memory integration, pass `with_cpu_memory=True`, register
`cpu_bus` against the reserved `SoCRegion`, and supply `cpu_memory_ready`.
Keep the host slave's decoded size consistent with `wb_slave_size` on the core.

The external-memory uRV wrapper is a pipelined Wishbone master: it issues one
request per cycle while the slave's `stall` is low and consumes in-order
responses. Through `WishboneClockCrossing` this degrades to one outstanding
access, as on SPEC-A7. With the same-clock external PHY, `cpu_memory_local=True`
instead keeps a `WRCPULocalMemory` inside the core: a single-cycle RAM
initialized from the firmware binary that restores the native one-fetch-per-cycle
uRV timing, with `cpu_memory_bus` as the host slave for loading and debug.
`add_white_rabbit` registers it as `wr_cpu_ram`. The Tang Mega 138K Pro target
uses this: through the SoC bus the SoftPLL interrupt consumed about half of
the CPU and the slave servo lost lock intermittently.

External build integrations can pass `wr_cpu_type`, `wr_cpu_variant`, and
`wr_cpu_memory` to `prepare_wr_environment`. The returned configuration
contains the validated selection; the caller must pass that selection to its
SoC constructor. Firmware lookup and rebuild select the corresponding CPU
profile. Defaults remain uRV with private memory.

## Acorn GTP phase-interpolator tuning

`python3 acorn_wr_nic.py --build --wr-refclk-tuning txpi` selects the optional
TXPI main-clock actuator. The default is `mmcm`; the helper clock always uses
its MMCM. TXPI leaves the 125 MHz QPLL reference fixed and tunes the WR
transmitter and its 62.5 MHz reference clock.

For another Artix-7 integration, enable `WhiteRabbitCore(with_txpi=True)` and
connect `WRTXPIBackend.command` to the core's main DAC data/load signals in
`wr_sys`. Connect the backend's `txpippmstepsize` to the core input of the same
name in `wr` (TXUSRCLK2). The backend crosses commands coherently and holds
the complete sign/magnitude word for two clocks. It starts neutral, retains
fractional phase across same-direction commands and clears unissued phase on
neutral, reversal or reset. GTX devices do not support this backend.

The GTP configuration uses the TX buffer, `TXOUTCLKSEL=010`,
`TXPI_SYNFREQ_PPM=001`, and a shared TXOUTCLK/TXUSRCLK2 clock, as required by
[XAPP589](https://docs.amd.com/go/en-US/xapp589-VCXO) and
[UG482](https://docs.amd.com/v/u/en-US/ug482_7Series_GTP_Transceivers).
Acorn selects `div_n=2`: for a 16-bit command, the mean step magnitude is
`abs(code - 32768) / 16384` per two-clock update. With `TXOUT_DIV=4` and a
20-bit datapath this gives a nominal 0.00596 ppm/code. Increasing the code
increases frequency. Recalibrate master trim when changing actuators; MMCM
trim values and loop qualification do not transfer automatically to TXPI.

## SPEC DAC control

The AD5683R CSRs keep the `force`, `load`, `value`, `current` register order.
`status` adds `ready` (bit 0), sticky `overflow` (bit 1), and applied `forced`
(bit 2). Check `ready` before writing `force` or writing 1 to `load`.
Set `force=1`, write `value`, then write `load=1` to submit one command.
The value and mode travel together through the clock crossing; later value
writes cannot change an already queued command. Writing `load=0` is optional
and has no effect. Writing `force=0` returns control to WR in queue order.

`current` holds the last code accepted by the serial DAC driver, including WR
commands, with synchronization latency before host readback. It does not read
the analog voltage or confirm SPI completion. The serial driver may coalesce
updates that arrive while it is busy. A write when `ready=0` is discarded and
sets `overflow`; reset clears the queue, applied host mode and overflow state.
