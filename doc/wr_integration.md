# Integrating White Rabbit in a LiteX design

`WhiteRabbitCore` is a `LiteXModule` that can be used without the PCIe NIC,
LiteEth SRAM replacement, or the NIC target's address map. It currently wraps
the existing Xilinx 7-series WR board implementation. Supply the board clocks,
SFP/I2C pads, optional UART/flash/one-wire pads, and oscillator tuning wiring.

```python
from litex.soc.integration.soc import SoCRegion
from litex_wr_nic.gateware.wr_core import WhiteRabbitCore

self.wr = WhiteRabbitCore(platform,
    cpu_firmware = "litex_wr_nic/firmware/spec_a7_wrc.bram",
    sfp_pads     = platform.request("sfp", 0),
    sfp_i2c_pads = platform.request("sfp_i2c", 0),
    serial_pads  = platform.request("serial"),
)
self.bus.add_slave("wr", self.wr.bus,
    SoCRegion(origin=0x20000000, size=0x01000000))
WhiteRabbitCore.add_sources(platform)
```

The board must supply `sys`, `clk_62m5_dmtd`, `clk_125m_gtp` and `clk10m_in`.
The wrapper produces `wr_sys` and `wr`, including their resets. Use Migen's
`ClockDomainsRenamer` on the component when a design needs different names.

| Interface | Clock | Contract |
| --- | --- | --- |
| `bus` | `sys` | 32-bit word-addressed host Wishbone slave |
| `cpu_bus` | `sys` | Optional 32-bit word-addressed CPU memory master |
| `sink` / `source` | `sys` | Application Ethernet byte streams |
| `dac_*_data`, `dac_*_load` | `wr_sys` | Oscillator tuning commands |
| `tm_seconds`, `tm_cycles`, `tm_time_valid`, `pps_*` | `wr` | PHY reference time and PPS |

For SoC memory, pass `with_cpu_memory=True`, register `cpu_bus` against a
reserved `SoCRegion`, and supply `cpu_memory_ready`. Keep readiness low until
memory initialization completes. VexRiscv additionally needs
`cpu_type="vexriscv"` and its matching `spec_a7_wrc_vexriscv.bram` firmware.
The current VexRiscv adapter supports the `lite` variant.

`add_white_rabbit(soc, ...)` registers the buses and preserves the historical
top-level attributes and CSR names. `LiteXWRNICSoC.add_wr_core(...)` delegates
to this helper, so existing targets need no changes. New designs can use
the component directly and choose their own hierarchy and CSR layout.

External build integrations can pass `wr_cpu_type`, `wr_cpu_variant`, and
`wr_cpu_memory` to `prepare_wr_environment`. The returned configuration
contains the validated selection; the caller must pass that selection to its
SoC constructor. Firmware lookup and rebuild select the corresponding CPU
profile. Defaults remain uRV with private memory.
