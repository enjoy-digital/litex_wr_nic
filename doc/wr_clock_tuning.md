# WR clock tuning backends

`WhiteRabbitCore.refclk_tuning` and `.dmtd_tuning` use `WRTuningInterface`:
an unsigned `data` code and a one-cycle `load` pulse, synchronous to **wr_sys**.
The existing `dac_refclk_*` and `dac_dmtd_*` signals remain aliases. These
commands do not belong to the PHY reference domain `wr`.

Both backends accept the same command record and build-time calibration:

```
output = saturate(center + offset + ((input - center) * polarity * gain >> gain_shift))
center = 2**(width - 1)
```

`polarity` is +1 or -1; `gain` is a positive integer, `gain_shift` a nonnegative
integer, and `offset` a signed code offset. Arithmetic is signed, right shifts
round toward negative infinity, and the result saturates to the unsigned
output range. Defaults are identity calibration. Calibration adds one cycle
of latency and exposes a `clipped` indication for the last command.

## External DAC

`WRDACBackend(width=16, cd="wr_sys", ...)` exposes `command`, `value` and
`load`. Connect its outputs to a DAC driver in the same domain:

```python
self.refclk_tuning = WRDACBackend(cd="wr_sys", polarity=1, offset=0)
self.comb += self.wr_core.refclk_tuning.connect(self.refclk_tuning.command)
self.refclk_dac = AD5683RDAC(platform, pads,
    load=self.refclk_tuning.load, value=self.refclk_tuning.value,
    gain=2, clk_domain="wr_sys")
```

SPEC-A7 uses this backend for both DACs. The AD5683R `gain` argument controls
the physical DAC gain independently of the digital calibration above. The
driver passes the correct VHDL generic, `g_enable_x2_gain`: previously
the misspelled generic was ignored and both DACs used the VHDL default x2
gain, including the DMTD DAC whose Python argument said `gain=1`.
SPEC-A7 now explicitly selects **x2 for both DACs**, and the driver defaults
to x2, preserving that effective configuration. `gain=1` is an explicit
opt-in to a different analog range and requires board/servo qualification.

The default digital calibration is identity: no gain, polarity or offset
change. This series does not retune the WR firmware PI coefficients.

Host override retains the `force`, `value`, `load` and `current` CSRs. Write
`force=1`, set `value`, then write `load=1` for each update; `load` now causes
one pulse even if the storage register remains high. Write `force=0` to
return control to WR. Mode/value/load travel together through a command
FIFO; `current` crosses back coherently. The serial driver can coalesce
updates while busy, so host override is for oscillator control, not arbitrary
sample playback. An integration needing every sample needs a paced DAC API.

## MMCM phase shifting

`WRMMCMBackend(cd_psclk, cd_command="wr_sys", width=16, div_n=0, ...)`
exposes `command`, `psen`, `psincdec` and **input `psdone`**. Connect all three
phase-shift signals to the MMCM and use its PSCLK domain for `cd_psclk`.
The Acorn and M2SDR targets explicitly instantiate this backend and connect
`PSDONE` for both MMCMs. The original `PSGen` implementation and its
`ctrl_data`/`ctrl_load` interface are unchanged: existing integrations do not
gain a new completion-input requirement just by updating the dependency.
An integration adopting `WRMMCMBackend` must connect `psdone` and use the
WR system domain for its command source. Declare the command and PSCLK
domains asynchronous in the platform constraints: tuning commands cross a
FIFO, and the disciplined clock can move relative to PSCLK. The Acorn and
M2SDR targets include this constraint. Synchronous backend paths still need
to meet the PSCLK period in the routed design.

For 7-series MMCMs, use `S7MMCM(..., fractional=False)` and enable fine phase
shifting on each tuned output. Fractional output division is incompatible
with fine phase shifting ([AMD UG472, pages 75 and 85](https://docs.amd.com/api/khub/documents/1kFbRqzm2fhwGy~cLQG2yA/content)).
Acorn and M2SDR explicitly use integer-only configurations for their WR
MMCMs. The nominal reference/DMTD frequencies remain 125/62.5 MHz; the
selected 1.5 GHz VCO gives a phase step of approximately 11.905 ps.

The nominal rate is `abs(code - center) / 2**(width + div_n + 3)` phase shifts
per PSCLK cycle. A code below center requests phase increment; a code above
center requests decrement. Each phase request is one PSCLK cycle. Direction
stays fixed until completion, and a new request waits for the previous
`PSDONE` and its deassertion. If the requested rate exceeds completion speed,
the rate saturates; it does not accumulate an unbounded backlog. Same-direction
updates preserve fractional phase. Command decoding uses one registered
PSCLK stage between the FIFO and accumulator to keep the 200 MHz path short;
the new magnitude applies when that stage loads the accumulator.
This includes repeated identical codes: refreshing a small correction must
not prevent it from accumulating a complete step. Neutral or direction
reversal clears unissued fractional phase without changing an outstanding
request.

The backend starts neutral and transfers complete tuning words through an
asynchronous FIFO. A single pending slot retains the newest command if the
FIFO fills; replacements increment `superseded`. The same `WRTuningCDC`
helper is available to other DAC integrations. Both FIFO sides and backend
state reset when either participating domain resets, with locally
synchronized reset release. Both sides remain reset until both clocks have
resumed, so stale FIFO pointers cannot replay pre-reset commands when one
clock was stopped. Fresh tuning commands are required afterward.

If `PSDONE` does not arrive within `timeout_cycles` (default 1024 PSCLK
cycles), a sticky fault stops further requests. Reset the backend **and MMCM**
together before resuming. Coherent CSRs expose busy/fault, completed steps
and superseded commands. A stopped PSCLK cannot advance this timeout, so a
board requiring independent clock-failure detection must monitor PSCLK from
a free-running domain.

Digital calibration does not make MMCM phase steps equivalent to an analog
VCXO. The board's MMCM configuration, oscillator polarity and sensitivity,
WR firmware servo settings, jitter, lock behavior and PPS accuracy still
require measurement. Keep a free-running clock for WR CPU/control and for
PSCLK; do not derive the control path from a clock it is trying to recover.

## Validation

`pytest -q test/test_wr_clock.py` checks full-range calibration, saturation,
coherent command bursts and replacement accounting, neutral startup,
completion backpressure, stable direction, timeout/reset behavior, and DAC
host override pulse semantics. Compatibility tests compare the default WR DAC
command stream with the previous registered path, decode the actual VHDL
driver's SPI initialization/update words, and check the MMCM's nominal rate
and polarity against the previous accumulator law. The legacy code-zero
magnitude overflow, non-neutral startup and unsafe overlapping requests are
not compatibility requirements; the corrected backend handles these cases.

`pytest -q test/test_wr_mmcm.py` adds generated-RTL simulations using Vivado's
`xvlog`, `xelab` and `xsim` (skipped if unavailable):

- Full 16-bit endpoints and one-LSB corrections in both directions, with
  repeated updates over more than four million PSCLK cycles. A deterministic
  varying command stream is checked against its independent code/time integral.
- Nominal, delayed, stretched, stale-high, absent and timeout-boundary
  completions; neutral/reversal while busy; completion/CSR accounting; sticky
  faults; stopped/restarted clocks and reset with a backed-up command FIFO.
- Actual `MMCME2_ADV`, clock-buffer and reset primitives from `unisims_ver`
  for Acorn's 200 MHz and M2SDR's 100 MHz inputs, with 125/62.5 MHz outputs
  and 200 MHz PSCLK. Checks cover the 12-cycle completion protocol, measured
  output phase, both reference outputs, wraparound in both directions, and
  reset during a shift followed by relock. Configuration checks separately
  reject fractional division; the vendor behavioral model does not enforce
  every hardware configuration restriction.

The phase comparisons allow 10 ps for simulator rounding. This is a digital
model tolerance, not a claim about hardware jitter or WR synchronization
accuracy. Reproduce both layers with:

```sh
pytest -q test/test_wr_clock.py test/test_wr_mmcm.py
```

`pytest -q test/test_wr_mmcm_formal.py` uses Yosys, SymbiYosys and Boolector
to prove the request protocol for arbitrary commands at the FIFO output and
arbitrary completion inputs, including missing/stuck/early completions and
resets. Both the default 16-bit/1024-cycle configuration and an 8-bit/divided
configuration are checked. The proof covers request exclusivity, one-cycle
PSEN pulses, direction stability during an outstanding shift, completion
accounting and sticky faults. Cover traces reach both shift directions,
neutral/reversal during a shift, simultaneous command/completion and a
timeout fault in the shorter configuration. Rate and code-to-direction
mapping remain covered by the independent simulation scoreboards above.
This proof does not model metastability or analog MMCM behavior.

### SPEC-A7 physical MMCM test

`bench/spec_a7_mmcm.py` builds a temporary SRAM image with four actual MMCMs:
100/200 MHz inputs, 125/62.5 MHz outputs and 200 MHz PSCLK. The SPEC-A7's -2
speed grade selects a different VCO from the Acorn/M2SDR -3 parts; each VCO
is exported in the CSR map and used in the output-frequency prediction.
This qualifies the backend on 7-series hardware, not WR servo lock or PPS
accuracy on an M2SDR.

From the repository root, with this checkout on `PYTHONPATH`:

```sh
PYTHONPATH=. python3 bench/spec_a7_mmcm.py --build
# Check build/spec_a7_mmcm/gateware/spec_a7_mmcm_timing.rpt and *_drc.rpt.
python3 litex_wr_nic/gateware/xilinx-bitstream.py \
    build/spec_a7_mmcm/gateware/spec_a7_mmcm.bit \
    build/spec_a7_mmcm/gateware/spec_a7_mmcm.bin
openFPGALoader --cable ft4232 --freq 20000000 \
    --bitstream build/spec_a7_mmcm/gateware/spec_a7_mmcm.bin
litex_server --jtag --jtag-config=/path/to/spec_a7_openocd.cfg
```

In another terminal:

```sh
python3 bench/spec_a7_mmcm_test.py --csr-csv build/spec_a7_mmcm/csr.csv
```

The runner checks the FPGA identifier before writing test CSRs. It measures
actual output-clock edges over 2**24 PSCLK cycles against the completed
signed phase shifts and selected VCO. It checks endpoints, one-LSB codes,
both polarities, neutral, every-cycle/slower command refresh, missing
completion, fault recovery and reset with the producer clock stopped.
An independent hardware monitor records overlapping/stretched requests,
direction changes while busy, unsolicited/wrong-latency completions and
loss of MMCM lock. Results are saved as JSON.

Like the main SPEC-A7 target, the bench targets the 50T resource map and uses
the repository's bitstream conversion for the connected 35T device.
Stop the LiteX server before restoring your working converted WR bitstream with
`openFPGALoader --cable ft4232 --bitstream /path/to/working_wr.bin`, then
check the WR console. These commands load SRAM; they do not update flash.

CPU/console tests on SPEC-A7 exercise the DAC integration; analog tuning range
and closed-loop WR operation need a WR peer.
