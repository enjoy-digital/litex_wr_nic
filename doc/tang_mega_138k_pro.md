# Tang Mega 138K Pro White Rabbit

This target runs White Rabbit on the **GW5AST-138B of the 138K Pro dock** and
synchronizes as WR master or slave with a SPEC-A7 or Acorn peer over SFP0
(or SFP1). It reuses LiteEth's raw Gowin SerDes and LiteX 8b/10b for the
PHY, keeps the upstream WR endpoint, PCS, timestamping and uRV firmware, and
adds two clock actuators from the dock hardware:

- **Main clock**: the dock's MS5351 clock generator provides the 100 MHz
  SerDes reference. The FPGA fine-tunes its PLL feedback fraction over I2C,
  which shifts the reference, the 125 MHz transmit clock and the WR reference
  time together.
- **Helper clock**: the GW5A PLL dynamic phase adjustment steps the 62.5 MHz
  DDMTD clock. A steady step rate is a frequency offset of up to ±195 ppm.

The WR CPU runs from a single-cycle LiteX RAM inside the core. This matters:
through the SoC bus the SoftPLL interrupt took about half of the CPU and the
slave servo lost lock every minute or so.

USB is required for programming and debug; there is no PCIe NIC,
external-reference input, persistent storage or SFP EEPROM access yet.

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
| `enjoy-digital/litex` | `dce79bf9abf6eb77e4f6e9358e11751f83051cab` plus [PR #2628](https://github.com/enjoy-digital/litex/pull/2628) (`GW5APLL.expose_dpa`) |
| `litex-hub/litex-boards` | `58634aac7029fd80dc7a8bbff1e5fbe22e141fb2` |
| `enjoy-digital/liteeth` | `0e2fbcf838d177ff1dad0595dba9f0f4aec931f1` ([PR #227](https://github.com/enjoy-digital/liteeth/pull/227)) |
| `enjoy-digital/litescope` | `6bf3b92f261c50b8c7c74947f84e692ae846f512` |

From the repository root:

```sh
python3 tang_mega_138k_pro_wr.py --build
```

This builds the pinned upstream WRPC firmware with its **8-bit PHY** profile
and read-only storage overlay. Its `tang_mega_138k_pro_wrc.*` filenames are
separate from the Acorn/SPEC 16-bit images. The firmware binary initializes
the 128 KiB CPU RAM, which the host also reaches at `wr_cpu_ram`
(`0x10000000`) for reloading without a rebuild. The WR HDL revision and
submodules are checked by the normal core builder.

The output directory contains `gateware/sipeed_tang_mega_138k_pro.fs` and
`csr.csv`. Keep them together. Use `--sfp 1` for SFP-1, with a separate
`--output-dir`. Only one SFP lane is instantiated per build.
`--skip-firmware-build` reuses the Tang firmware already built in this
checkout, `--with-analyzer` adds a LiteScope capture of the raw and decoded
RX symbols on the recovered clock (see below). Retain generated reports and
captures under `build/`, outside commits.

The build runs with zero setup/hold violations on Gowin 1.9.12 and uses about
8% of the logic and 26% of the block RAM of the GW5AST-138.

## Dock clock generator

The tuner assumes the documented dock configuration of MS5351 PLL0
(`pll_switch 0`): PLLA at 25 MHz × 36 = 900 MHz and MS0/MS1 = /9 for the
100 MHz differential reference on Q1 REFCLK1, i.e. the 100 MHz line of the
[Sipeed clock setup guide](https://github.com/sipeed/TangMega-138KPro-example/blob/main/sfp%2B/docs/SET_5351.md).
The FPGA only rewrites PLLA's P1/P2/P3 registers: bit 20 of its 21-bit code
selects ×36 or ×35 127/128 and the low bits are the fraction with a 2**20 - 1
denominator, so the tuning range is ±1/(128 × 36) = ±217 ppm in steps of
about 0.2 ppb. No PLL reset is needed for these updates and the SerDes stays
locked through them. Every 256th update rewrites all eight registers.

The dock's debugger MCU programs the MS5351 at power-up from its saved
configuration; it does not touch the chip afterwards, and the FPGA never
changes the integer multiplier or the output dividers. If a previous session
left the chip in another state, check or restore it through the target's
host I2C path. The tool drives the FPGA UART directly (stop `litex_server`
first): the MS5351 aborts transfers whose clock edges are milliseconds apart,
so the bit-bang states are sent as fixed-address UARTBone bursts.

```sh
python3 tools/tang_ms5351.py --csr-csv build/tang_mega_138k_pro_wr/csr.csv --uart-port /dev/serial/by-id/FPGA_UART
python3 tools/tang_ms5351.py --csr-csv build/tang_mega_138k_pro_wr/csr.csv --uart-port /dev/serial/by-id/FPGA_UART --restore
```

`--restore` programs ×36 and /9, resets the PLL and re-enables the outputs;
the SerDes reference is interrupted for a few milliseconds. The FPGA reaches
the chip through the dock's 8-way I2C multiplexer (channel 3); the same host
path with `main_i2c_sel` selects the SFP0/SFP1 EEPROMs on channels 0/1.

## USB and optical test

1. Power the Pro dock and connect its programming/debug USB.
2. Connect the selected SFP port to a WR peer, such as SPEC-A7 SFP0/J12,
   using compatible modules and fiber.
3. Select the Tang programming probe and load the SRAM image:

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

   Replace `FPGA_UART` with the actual stable USB path (the second interface
   of the Sipeed debugger). Press Enter for the prompt; the finite console
   FIFO can lose earlier boot output. The physical UART carries Wishbone
   traffic, so a plain serial terminal is not the WR console. GW5 JTAGBone is
   not supported by this target.

5. Assign a unique local MAC, for example `mac set 02:00:00:00:13:80`, before
   protocol experiments. Do not reuse the peer's default MAC. The
   initialization script only runs `ptp stop`, but WRPC restarts PTP on
   link-up; stop it explicitly for diagnostics.

`phy_status` bits 0–4 report TX PLL lock, RX CDR lock, comma alignment,
RX valid and PHY ready. `{ref,dmtd,rx}_clk_freq_value` report frequency in
Hz, updated approximately once a second against the 50 MHz system oscillator:
expect about 125 MHz for `ref`/`rx` and 62.5 MHz for `dmtd`. These counters
are not an independent frequency reference; `ref`/`rx` measured against the
same gate show the frequency offset between the Tang reference and the peer.
LEDs 0, 1 and 2 expose the WR link, activity and PPS signals.

## Master and slave operation

With the link up and PTP stopped on both boards:

- **Tang slave, peer master**: on the peer `mode master` then `ptp start`; on
  the Tang `mode slave` then `ptp start`.
- **Tang master, peer slave**: on the Tang `mode master` then `ptp start`; on
  the peer `mode slave` then `ptp start`.

`stat` on the slave reports the servo state (`ss`), the reported offset
(`cko`, in ps) and the round-trip delay (`crtt`). `pll stat` reports the
SoftPLL: `HL1` helper locked, `MFL1 MPL1` main frequency and phase locked,
`HY`/`MY` the helper/main commands.

The actuator CSRs describe the servo commands and their application:

| Register | Meaning |
| --- | --- |
| `main_tuning_command` | Latest 16-bit WR main-clock command (32768 neutral) |
| `main_tuning_center` | MS5351 code for a neutral command; nominal `0x100000` |
| `main_tuning_shift` | MS5351 codes per command step, as a power of two |
| `main_tuning_code`, `main_tuning_updates`, `main_tuning_errors` | Applied code, completed and unacknowledged I2C updates |
| `main_tuning_status` | I2C update in progress; fractional configuration valid |
| `main_tuning_control` | Clear `enable` to release the I2C bus to the host master |
| `helper_tuning_command`, `helper_tuning_steps` | Latest helper command and issued PLL phase steps |

The main servo spans ±32768 × 2**shift codes around the center. With the
default `--main-shift 1` this is ±14 ppm in 0.4 ppb steps, so the center
must absorb the frequency offset between the Tang's 25 MHz crystal and the
peer; the bench boards differ by about 49 ppm. Measure it from
`ref_clk_freq_value` and `rx_clk_freq_value` with the main loop at a known
code, or simply from a saturated command (`MY` at 0 or 65535): move the
center by `(offset_ppm × 1e-6) / (217e-6 / 2**20)` codes, about 4830 codes
per ppm, and pass it with `--main-center` or write `main_tuning_center` at
run time. The same center serves the master role, where the Tang free-runs
at the neutral command. Shifts of 3 and 4 remove the need for a center on
this bench but did not phase-lock with the default SoftPLL gains: the I2C
update latency leaves too little margin at that loop gain.

Both roles were tested against SPEC-A7 (SFP0/J12) on this bench with the
bench-calibrated center `810000`:

- Tang slave, `shift=1`: the WR handshake completes with RX timestamp
  calibration, the servo reaches `TRACK_PHASE` within about 30 s and holds
  it. Over a 10-minute run the servo-reported offset had a mean of −0.1 ps
  and a standard deviation of 2.5 ps (extremes −8/+13 ps), the round-trip
  delay stayed within 30 ps, with no helper or main lock loss and no I2C
  errors at about 3900 main-clock updates per second. With `shift=0` a
  15-minute run tracked similarly (σ 6.7 ps, one −66 ps transient).
- Tang master, SPEC-A7 slave: SPEC-A7 tracks the Tang reference in
  `TRACK_PHASE`; over 5 minutes its reported offset had a mean of −0.8 ps
  and a standard deviation of 1.2 ps (extremes −3/+2 ps).

These are servo statistics. Absolute accuracy still requires the receive
latency/bitslide calibration, board and module delay and asymmetry
calibration, and independent PPS measurements, as for the other boards.

## Symbol capture

For a symbol capture, use `LiteScopeAnalyzerDriver` over the same server with
`analyzer.csv` from a `--with-analyzer` build. For example, from the
repository root:

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

## Remaining work

- Characterize/bypass the RX FIFO and hardware comma aligner, obtain the actual
  bitslide, and prove repeatable TX/RX latency across resets. `rx_bitslide=0` is
  a placeholder. PHY loopback and exhaustive 8b/10b error/disparity checking also
  need implementation; the adapter currently uses LiteX's basic decoder check.
- Route the dock's SFP management to the WR firmware for module
  identification and the calibration database (`sfp match`), and choose an
  accessible PPS output for independent measurements.
- Qualify the loop with a wider main-clock range (`main_tuning_shift`) and
  measure PPS alignment/jitter independently with board/module delay and
  asymmetry calibration.

The only new VHDL is a flat record adapter around `xwrc_board_common`. The raw
SerDes, codec, resets, clocking and debug logic reuse LiteEth/LiteX. Portable
source preparation selects the inferred upstream shift register and stages
small UART/RAM compatibility changes without changing the Xilinx source list.
