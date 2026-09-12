# Acorn / SPEC-A7 USB bench

This profile connects Acorn SFP0 to **SPEC J12 / SFP0**, with UART and JTAG
through each board's FT4232H USB interface. It uses uRV/private 128 KiB RAM,
Acorn MMCM tuning, and SPEC RefClk DAC ×2 / DMTD DAC ×1.

## Build and connect

Install the dependency revisions in [wr_bench_sources.json](wr_bench_sources.json),
Vivado, GHDL, pyserial, OpenOCD and openFPGALoader. Build sequentially because
the targets share the generated firmware filename:

```sh
python3 tools/build_wr_bench.py --board acorn --output build/bench-acorn
python3 tools/build_wr_bench.py --board spec --output build/bench-spec
```

The helper verifies dependency revisions, builds firmware from the pinned WRPC
sources, disables firmware SPI writes/erases, and keeps calibration updates in
RAM. It saves the programming image, firmware, CSR map, hashes and timing
report under the output directory.

Copy `doc/wr_bench.example.json` to `build/wr-bench.json`. Paths are relative to
the configuration file. Replace USB locations and UART paths; UART uses
FT4232H interface 2 and JTAG uses channel 0. Derive each board's `master_trim`
using the procedure below and add that integer field to its local configuration.

Stop managed servers before programming. Resolve each USB location under
`/sys/bus/usb/devices/` to its `busnum` and `devnum`, then load one board at a
time using those numbers as `BUS:DEVICE`:

```sh
python3 tools/wr_jtag.py --config build/wr-bench.json stop
openFPGALoader --cable ft4232 --busdev-num BUS:DEVICE --freq 5000000 --bitstream build/bench-acorn/gateware/sqrl_acorn.bit
openFPGALoader --cable ft4232 --busdev-num BUS:DEVICE --freq 5000000 --bitstream build/bench-spec/gateware/spec_a7_wr_nic.bin
```

These commands program SRAM. For the physical XC7A35T SPEC, use the converted
`.bin`, not the raw 50T `.bit`. Keep each image's matching `csr.csv`.

## Derive the master trim

Through UART, stop PTP and the PLL loops on both boards and hold both actuators
at midscale:

```text
ptp stop
pll init 0 0 0
pll sdac 0 32768
pll sdac -1 32768
freqmon rx
freqmon
```

For each board in turn, leave the peer at midscale and sweep main channel 0
with `pll sdac 0 CODE`. At each point issue `freqmon rx`, allow the frequency
to settle, then read `freqmon`. Save CSV columns `board,code,ref_hz,rx_hz`, with
board names `acorn` and `spec`. Include Acorn code 32768, SPEC code 0, at least
three codes in each linear region, and SPEC's upper plateau. Restore midscale
before changing boards. Inspect the sweep to select `LINEAR_MAX` and
`PLATEAU_MIN` for this SPEC:

```sh
python3 tools/wr_trim.py build/main-sweep.csv --spec-linear-max LINEAR_MAX --spec-plateau-min PLATEAU_MIN > build/trim-fit.json
```

The fit uses reciprocal REF/RX ratios and centers SPEC's usable range. Copy
its per-board `master_trim` values into the local configuration. These are
relative measurements; an independent reference is needed for absolute
frequency calibration. The reported slopes use ppm/code.

## Check both roles

```sh
python3 tools/wr_jtag.py --config build/wr-bench.json start
python3 tools/wr_qualify.py --config build/wr-bench.json --master-board acorn --configure --duration 120 --output build/check-acorn-master
python3 tools/wr_qualify.py --config build/wr-bench.json --master-board spec --configure --duration 120 --output build/check-spec-master
python3 tools/wr_jtag.py --config build/wr-bench.json stop
```

`--configure` applies the selected master's trim and restarts PTP. The checker
waits for WR acquisition before starting the requested tracking interval. It
checks UART GUI frames, JTAG diagnostics, roles, PLL locks, phase tracking,
time/servo progress and RX errors. Internal tracking checks do not establish
independent PPS alignment or jitter.

Keep the manager's state directory across interruptions: it records process
ownership for cleanup and shares a lock with the checker. Captures and build
outputs belong outside version control.
