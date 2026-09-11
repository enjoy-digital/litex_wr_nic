# Acorn / SPEC-A7 USB bench

This profile uses Acorn CLE215+ on the Acorn Baseboard Mini, SPEC-A7 with
the physical XC7A35T, Acorn SFP0 connected to **SPEC J12 / SFP0**, and each
board's FT4232H UART and JTAG interfaces. PCIe and host Ethernet are not needed.
The WR CPU is uRV with private 128 KiB RAM. SPEC uses RefClk DAC ×2 and DMTD
DAC ×1; Acorn uses the qualified MMCM implementation.
The measurements below describe the connected pair on 2026-09-11.

## Build from pinned sources

Use a separate checkout of this branch. The build helper verifies the clean
Python dependency revisions in [wr_bench_sources.json](wr_bench_sources.json).
Install those checkouts in a virtual environment, for example:

```python
import json, pathlib, subprocess, sys
pins = json.loads(pathlib.Path("doc/wr_bench_sources.json").read_text())
root = pathlib.Path("build/deps")
root.mkdir(parents=True, exist_ok=True)
for name, pin in pins["python"].items():
    path = root / name
    subprocess.run(["git", "clone", pin["url"], str(path)], check=True)
    subprocess.run(["git", "-C", str(path), "checkout", "--detach", pin["revision"]], check=True)
    subprocess.run([sys.executable, "-m", "pip", "install", "-e", str(path)], check=True)
```

Use Vivado 2024.1 and GHDL (the bench uses 4.0.0-dev, gbd6c861b1), with
`pyserial` for the UART tools and OpenOCD/openFPGALoader for JTAG access.
The firmware helper obtains the pinned `riscv-11.2-small` GCC 11.2 toolchain.
Official WR sources now live under `https://gitlab.com/ohwr/project/`.
The build initializes and checks recursive submodules; WRPC is built separately
from the older WRPC submodule carried inside wr-cores.

Run the builds sequentially because the target-specific firmware filename is
shared. Each output preserves its firmware, CSR map, source revision, hashes
and timing report. Build outputs are not intended for version control.

```sh
python3 tools/build_wr_bench.py --board acorn --output build/bench-acorn
python3 tools/build_wr_bench.py --board spec --output build/bench-spec
```

Firmware SPI writes/erases return read-only errors; automatic calibration
updates remain in RAM. Existing flash configuration can still be read.
The ordinary firmware build remains writable unless `--read-only-storage`
is selected. Reusing a firmware checkout restores the owned source overlays
before applying the selected profile.

The relevant earlier clock work from #77 is included through the clean
integration #85, followed by DAC control #86. #77 merged into
`wr-fabric-flow-control`, not main. This profile does not require merging that
whole branch or the experimental PICXO alternative. LitePCIe #186 and LiteX
#2590 are merged; the dependency pin also includes LiteX cleanup #2591.
CPU diagnostic-map and SFP-storage fixes remain explicit overlays on the
official WR sources, so an unmerged contribution mirror is not a build input.
#84 retains the historical qualification material.

## Connect, load and observe

Copy `doc/wr_bench.example.json` to `build/wr-bench.json` and edit it.
**All relative paths are relative to the configuration file.** Replace the
UART paths and USB locations with the actual connections. Select the FT4232H
UART interface 2; JTAG uses channel 0. The example master trims identify this
measured pair, not every board of
these models. Preserve module serials and temperature/reference details in
your local measurement record.

Stop managed servers before programming:

```sh
python3 tools/wr_jtag.py --config build/wr-bench.json stop
```

Resolve each configured USB location under `/sys/bus/usb/devices/` to its
`busnum` and `devnum`, and verify VID/PID `0403:6011`. Load one selected board
at a time with openFPGALoader, using those numbers as `BUS:DEVICE`:

```sh
openFPGALoader --cable ft4232 --busdev-num BUS:DEVICE --freq 5000000 --bitstream build/bench-acorn/gateware/sqrl_acorn.bit
openFPGALoader --cable ft4232 --busdev-num BUS:DEVICE --freq 5000000 --bitstream build/bench-spec/gateware/spec_a7_wr_nic.bin
```

These commands program SRAM. SPEC's target builds for XC7A50T and converts
the image for the physical XC7A35T; use the resulting **`.bin`**, not the raw
50T `.bit`. Retain the matching `csr.csv` from each build.

```sh
python3 tools/wr_jtag.py --config build/wr-bench.json start
python3 tools/wr_qualify.py --config build/wr-bench.json --master-board acorn --configure --duration 120 --output build/check-acorn-master
python3 tools/wr_qualify.py --config build/wr-bench.json --master-board spec --configure --duration 120 --output build/check-spec-master
python3 tools/wr_jtag.py --config build/wr-bench.json stop
```

The manager binds separate local server/stream ports, verifies FPGA identity
and the WR host signature, and limits startup to three attempts. Ownership
records include Linux boot ID, process group and PID start times. Cleanup can
recover a recorded OpenOCD child after its server was killed; it refuses to
signal an unverified/reused process group. Preserve the state directory across
interruptions. LiteX #2591 handles normal SIGTERM cleanup; a later manager
invocation handles recorded SIGKILL orphans. The shared bench lock prevents
managed reconnects during qualification; physical UART opens are exclusive.

The checker requires complete UART GUI frames and coherent JTAG diagnostics,
WR extension, correct roles, PLL locks, phase tracking, advancing time and
servo updates, and unchanged RX error counts. Acquisition has a separate
timeout; the requested duration starts only after every check passes.
An unnormalized PPS-counter snapshot can occur during initial clock
adjustment. It is retained as invalid acquisition data and cannot qualify
tracking. Invalid timestamps during the tracking interval still fail the run.

## Measure and set the master trim

At Acorn code 32768, SPEC's main oscillator is about 26 ppm faster. Even SPEC
code 0 remains about **1.4 ppm faster**: the slave reaches its low rail before
matching Acorn. SPEC's main frequency also plateaus above roughly code 40000.
Consequently digital midscale is not the center of its usable frequency range.

The measured pair gave the following approximate ranges and sensitivities:

| Actuator | Measured full span | Linear sensitivity |
| --- | ---: | ---: |
| Acorn main MMCM | 298 ppm | 0.00454 ppm/code |
| Acorn helper MMCM | 297 ppm | 0.00454 ppm/code |
| SPEC main, DAC ×2 | 29.6 ppm | 0.000754 ppm/code, before plateau |
| SPEC helper, DAC ×1 | 176 ppm | 0.00268 ppm/code |

These are reciprocal frequency measurements using each peer's recovered RX
clock, not an independent absolute reference. Counter resolution is about
0.2 ppm; sequential sweeps can also include oscillator drift. The plateau is
consistent with DAC output headroom, but its voltage cause is not measured.

To reproduce the main-clock calibration, keep both CPUs running, stop PTP and
the PLL loops on both boards, and hold both main/helper commands at 32768:

```text
ptp stop
pll init 0 0 0
pll sdac 0 32768
pll sdac -1 32768
freqmon rx
freqmon
```

The first `freqmon` read primes the firmware's cached monitor configuration.
For each board in turn, leave the peer at 32768 and sweep main channel 0 over
`0, 4096, 8192, 16384, 24576, 32768, 35000, 38500, 40960, 49152, 57344, 65535`.
At each point issue `pll sdac 0 CODE`, then `freqmon rx`, wait at least three
seconds, and read `freqmon`. Record CSV columns `board,code,ref_hz,rx_hz`, using
board names `acorn` and `spec`. Restore the swept main to 32768 before changing
boards. Channel -1 and the DMTD readout can separately characterize the helper.

```sh
python3 tools/wr_trim.py build/main-sweep.csv > build/trim-fit.json
```

The fit normalizes REF by RX, anchors Acorn's sweep to its measured 32768
point, and chooses the midpoint between SPEC's low endpoint and averaged upper
plateau. It fits SPEC only through code 38500. Inspect the measured curve and
adjust the linear/plateau thresholds for other hardware. The fitted target is
about +16.16 ppm relative to this Acorn at 32768, giving **Acorn 36301 / SPEC
19677**. Put the resulting codes in the corresponding local `master_trim`
fields. `wr_qualify --configure` applies the selected master's value with
`pll sdac 0 CODE`; the slave keeps automatic control. Reapply after reset.

Both master directions passed 120 seconds of uninterrupted tracking at these
settings. With Acorn master, SPEC main/helper settled near 19500. The helper
at ×1 then has roughly 52 ppm to its measured low endpoint and 123 ppm to its
high endpoint. This supports static actuator headroom at the measured condition;
it does not establish phase margin or temperature coverage. Do not change PI
coefficients solely from these static slopes. Temperature characterization
requires recorded board temperatures, repeated sweeps/operating points and
settling measurements across the intended range.

## Physical work still required

Acorn's FPGA SCL/SDA pad readbacks followed driven high/low combinations, but
the module NACKed address 0x50. SPEC ACKed and identified Tyco Electronics
2127931-2, serial 11274837; its calibration database was empty. Acorn's all-FF
EEPROM read is invalid, despite its optical data link working.

The [baseboard schematic](https://github.com/enjoy-digital/litex-acorn-baseboard/blob/master/hardware/acorn-baseboard-mini-2022-06-06.pdf)
routes SFP0 SCL through **JP1** and SDA through **JP4**, via the PCA9306 level
shifter (FPGA side 1.8 V, SFP side 3.3 V). JP5/JP6 serve SFP1. Confirm the
actual board revision and jumper continuity, then observe SCL/SDA on both
sides during address 0x50. Pad readback alone does not prove module-side
continuity. After correcting the path, verify address ACK, EEPROM checksums,
vendor/part/serial with `sfp info`, and calibration lookup with `sfp match`.
The same routing requirement is recorded in [bring-up issue #2](https://github.com/enjoy-digital/litex_wr_nic/issues/2).

Independent PPS alignment and jitter remain unmeasured. Compare the actual
PPS outputs on two channels of a scope/time-interval instrument; characterize
channel and cable skew by applying one source to both and swapping channels.
Use compatible input levels/termination, retain edge timestamps, and report
mean delay, standard deviation, peak-to-peak spread, sample count and reference
uncertainty for both role directions. Account for the physical output paths:
Acorn drives 3.3 V GPIO H5 (`debug[0]`), requiring a high-impedance probe or
appropriate buffer; SPEC's default bypasses macro/coarse delay but still has its
external fine-delay/output path. A repeatable cable/channel correction must
precede board delay calibration. SFP TX/RX fixed delays and fiber asymmetry
need measured calibration; one PPS comparison cannot uniquely determine all
of them. Internal WR servo offsets are not absolute PPS accuracy measurements.
