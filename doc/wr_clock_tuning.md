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
driver now passes the correct VHDL generic, `g_enable_x2_gain`: previously
the misspelled generic left the VHDL default gain enabled. The requested
SPEC-A7 gains are x2 for RefClk and x1 for DMTD. Recheck the DMTD tuning range
and WR servo behavior on a connected link when qualifying this correction.

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
The Acorn target connects `PSDONE` for both MMCMs. `PSGen` remains a
compatibility constructor with `ctrl_data`/`ctrl_load` aliases; external
users must add the `psdone` connection when upgrading.

The nominal rate is `abs(code - center) / 2**(width + div_n + 3)` phase shifts
per PSCLK cycle. A code below center requests phase increment; a code above
center requests decrement. Each phase request is one PSCLK cycle. Direction
stays fixed until completion, and a new request waits for the previous
`PSDONE` and its deassertion. If the requested rate exceeds completion speed,
the rate saturates; it does not accumulate an unbounded backlog. A new code
clears the fractional accumulator without changing an outstanding request.

The backend starts neutral and transfers complete tuning words through an
asynchronous FIFO. A single pending slot retains the newest command if the
FIFO fills; replacements increment `superseded`. The same `WRTuningCDC`
helper is available to other DAC integrations. Both FIFO sides and backend
state reset when either participating domain resets, with locally
synchronized reset release. Fresh tuning commands are required afterward.

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
host override pulse semantics. CPU/console tests on SPEC-A7 exercise the DAC
integration; analog tuning range and closed-loop WR operation need a WR peer.
