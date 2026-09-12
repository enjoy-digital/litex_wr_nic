# Integrating White Rabbit in a LiteX design

`WhiteRabbitCore` is a `LiteXModule` that can be used without the PCIe NIC,
LiteEth SRAM replacement, or the NIC target's address map. It currently wraps
the existing Xilinx 7-series WR board implementation. HDL sources are added
automatically by `do_finalize()`; no separate `add_sources()` call is needed.

The examples below belong inside an existing SoC constructor, after its
platform, bus and board clocking have been created. Choose one CPU setup.
They use SPEC-A7 resource names and firmware; adapt pads and firmware target
for another board. Your SoC supplies the host bus master (for example a LiteX
CPU or JTAGBone).

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

Both helper examples create `self.wr_core` and retain the historical
attributes and CSR names. `wr` is a local reference to that module; do not
register it again as another SoC submodule. `LiteXWRNICSoC.add_wr_core(...)`
delegates to the same helper. `wb_slave_region` preserves the supplied region's
attributes and takes precedence over the legacy `wb_slave_origin` and
`wb_slave_size` arguments, which remain supported. LiteX rounds the decoded
region to a power of two; the core uses that same size for local addressing.

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
| `sink` / `source` | `sys` | Application Ethernet byte streams |
| `dac_*_data`, `dac_*_load` | `wr_sys` | Oscillator tuning commands |
| `tm_seconds`, `tm_cycles`, `tm_time_valid`, `pps_*` | `wr` | PHY reference time and PPS |

For direct SoC-memory integration, pass `with_cpu_memory=True`, register
`cpu_bus` against the reserved `SoCRegion`, and supply `cpu_memory_ready`.
Keep the host slave's decoded size consistent with `wb_slave_size` on the core.

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
