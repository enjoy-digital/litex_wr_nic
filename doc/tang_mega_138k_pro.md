# Tang Mega 138K Pro WR bring-up

This target prepares the **GW5AST-138B on the 138K Pro dock** for WR PHY
development. It includes the upstream WR endpoint, PCS, timestamp logic and
uRV firmware, with LiteX UARTBone, firmware RAM and optional LiteScope.
It requires USB for programming/debug and fiber for the optical test.

This is a console/link bring-up target. **WR master/slave synchronization is
not implemented yet:** the main and helper clock commands are only latched
in diagnostic CSRs, and receive latency is uncalibrated. Firmware starts with
`ptp stop`. There is no PCIe NIC, external-reference input or persistent storage.

## Build

Use Gowin **1.9.12** with GW5AST-138B support, **GHDL 6.0.0** with synthesis
support, and the project's RISC-V 11.2 toolchain. Put `ghdl` and `gw_sh` on
`PATH`. Older GHDL development snapshots can crash during WR conversion;
[GHDL release binaries](https://github.com/ghdl/ghdl/releases/tag/v6.0.0)
include an Ubuntu 24.04 build. GHDL converts the portable WR VHDL to Verilog;
Gowin synthesizes and routes it together with the LiteX logic and uRV RTL.

Install these Python source revisions in an isolated environment. The older
Acorn/SPEC dependency manifest predates the required Gowin support.

| Repository | Revision |
| --- | --- |
| `m-labs/migen` | `4c2ae8dfeea37f235b52acb8166f12acaaae4f7c` |
| `enjoy-digital/litex` | `9478c44e7b9bd5db1bf7d47119b86a4492d003ee` |
| `litex-hub/litex-boards` | `15cdfe4cde89b383e0ec752092dab66d24695b98` |
| `enjoy-digital/liteeth` | `d9106980ea5c2c112fd0264d2471fa25302062e8` ([PR #224](https://github.com/enjoy-digital/liteeth/pull/224)) |
| `enjoy-digital/litescope` | `6bf3b92f261c50b8c7c74947f84e692ae846f512` |

From the repository root:

```sh
python3 tang_mega_138k_pro_wr.py --build --with-analyzer
```

This builds the pinned upstream WRPC firmware with its **8-bit PHY** profile
and read-only storage overlay. Its `tang_mega_138k_pro_wrc.*` filenames are
separate from the Acorn/SPEC 16-bit images. uRV accesses 128 KiB of LiteX RAM
through the existing WR CPU memory bridge. The WR HDL revision and submodules
are checked by the normal core builder.

The output directory contains `gateware/sipeed_tang_mega_138k_pro.fs`,
`csr.csv` and `analyzer.csv`. Keep them together. Use `--sfp 1` for SFP-1,
with a separate `--output-dir`. Only one SFP lane is instantiated per build.
`--skip-firmware-build` reuses the Tang firmware already built in this checkout.
Retain generated reports and captures under `build/`, outside commits.

## USB and optical test

1. Power the Pro dock and connect its programming/debug USB. Keep its MS5351
   **PLL0 at 100 MHz differential on Q1 REFCLK1**. Follow the
   [Sipeed clock setup guide](https://github.com/sipeed/TangMega-138KPro-example/blob/main/sfp%2B/docs/SET_5351.md)
   if its configuration has changed. The debugger's PLL configuration console
   is distinct from the FPGA UART. No MS5351 reconfiguration is performed by
   this target.
2. Connect the selected SFP port to a known 1000BASE-X peer, such as SPEC-A7
   SFP0/J12, using compatible modules and fiber.
3. Select the Tang programming probe and load the matching SRAM image:

   ```sh
   openFPGALoader --scan-usb
   openFPGALoader -c ft2232 --usb-serial-num TANG_PROBE_SERIAL \
       build/tang_mega_138k_pro_wr/gateware/sipeed_tang_mega_138k_pro.fs
   ```

   Replace `TANG_PROBE_SERIAL` with the Tang probe's serial, particularly when
   Acorn/SPEC probes remain connected. Use `--busdev-num BUS:DEVICE` instead
   if the probe has no unique serial.

4. Start UARTBone on the FPGA UART and open the WR console in another terminal:

   ```sh
   litex_server --uart --uart-port /dev/serial/by-id/FPGA_UART --uart-baudrate 115200
   litex_term crossover --csr-csv build/tang_mega_138k_pro_wr/csr.csv
   ```

   Replace `FPGA_UART` with the actual stable USB path. Press Enter for the
   prompt; the finite console FIFO can lose earlier boot output. The physical
   UART carries Wishbone traffic, so a plain serial terminal is not the WR
   console. GW5 JTAGBone is not supported by this target.
5. Keep PTP stopped while checking `help`, `ver`, `mac`, `stat` and link
   recovery after unplug/replug. Assign a unique local MAC, for example
   `mac set 02:00:00:00:13:80`, before protocol experiments. Do not reuse the
   peer's default MAC.

`phy_status` bits 0–4 report TX PLL lock, RX CDR lock, comma alignment,
RX valid and PHY ready. Check these through a LiteX `RemoteClient` using the
matching CSR CSV. `{ref,dmtd,rx}_clk_freq_value` report frequency in Hz,
updated approximately once a second against the system oscillator. Expect
approximately 125 MHz; these counters are not an independent frequency reference.
`main_main_dac` and `main_helper_dac` show firmware requests, not applied tuning.
LEDs 0, 1 and 2 expose the WR link, activity and PPS LED signals.

For a symbol capture, use `LiteScopeAnalyzerDriver` over the same server with
`analyzer.csv`. For example, from the repository root:

```python
import time
from litex import RemoteClient
from litescope import LiteScopeAnalyzerDriver

output = 'build/tang_mega_138k_pro_wr'
bus = RemoteClient(csr_csv=f'{output}/csr.csv')
bus.open()
analyzer = None
try:
    assert bus.regs.phy_status.read() & 0x17 == 0x17, 'PHY is not ready'
    analyzer = LiteScopeAnalyzerDriver(bus.regs, 'analyzer', f'{output}/analyzer.csv')
    analyzer.configure_group(0)
    analyzer.configure_subsampler(1)
    analyzer.configure_trigger()
    analyzer.run(offset=0, length=1024)
    deadline = time.monotonic() + 5
    while not analyzer.done():
        if time.monotonic() > deadline:
            raise TimeoutError('No capture completion; check the recovered RX clock')
        time.sleep(0.05)
    analyzer.upload()
    analyzer.save(f'{output}/rx.vcd')
finally:
    try:
        if analyzer is not None:
            analyzer.clear()
    finally:
        bus.close()
```

The analyzer runs on the recovered RX clock. Group 0 exposes raw symbols and
RX validity/empty/alignment; select group 1 for decoded data/K/error and ready.
A stopped RX clock cannot complete a capture. Narrow probe groups and a
block-RAM-sized trigger FIFO avoid a large distributed-memory read mux.
The subsampler supports factors 1–16.

## Work required for synchronization

- Implement coherent main/helper clock control, including the helper's DDMTD
  frequency offset. The fabric clocks currently come from the 50 MHz oscillator;
  the TX reference comes from the dock's independent MS5351. Investigate GW5 PLL
  dynamic phase control and the SerDes reference path before choosing an actuator.
- Characterize/bypass the RX FIFO and hardware comma aligner, obtain the actual
  bitslide, and prove repeatable TX/RX latency across resets. `rx_bitslide=0` is
  a placeholder. PHY loopback and exhaustive 8b/10b error/disparity checking also
  need implementation; the adapter currently uses LiteX's basic decoder check.
- Check the dock's SFP management routing, add EEPROM identification/calibration
  access and choose an accessible PPS output. These pads are not exposed here.
- Then test both WR roles against Acorn/SPEC, tune the loops and measure PPS
  alignment/jitter independently with board/module delay and asymmetry calibration.

The only new VHDL is a flat record adapter around `xwrc_board_common`. The raw
SerDes, codec, resets, clocking and debug logic reuse LiteEth/LiteX. Portable
source preparation selects the inferred upstream shift register and stages
small UART/RAM compatibility changes without changing the Xilinx source list.
