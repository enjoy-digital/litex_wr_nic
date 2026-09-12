# SoftPLL capture tools

Use the [USB bench setup](wr_usb_bench.md) and its pinned dependencies. Build
both targets sequentially with the optional trace profile:

```sh
python3 tools/build_wr_bench.py --board acorn --pll-trace-decimation 16 --output build/trace-acorn
python3 tools/build_wr_bench.py --board spec --pll-trace-decimation 16 --output build/trace-spec
```

This enables the upstream SoftPLL host FIFO and XADC temperature/supply CSRs.
On SPEC, it omits PCIe NIC/PTM and the time generator to fit the XC7A35T RAM
budget; the PCIe PHY/QPLL clock arrangement, WR, UART, JTAGBone and PPS remain.
Update the local configuration with the trace images and their CSR maps, and
load SRAM as described in the bench setup. SPEC uses the converted `.bin`.

For native capture, the host JTAG bridge needs the TCP latency fix from
[LiteX #2592](https://github.com/enjoy-digital/litex/pull/2592), commit
`84368c06e34a2b703afd2f274459636fe0de79a8`. Set the configuration's `pythonpath`
to a checkout containing that fix; FPGA builds use the dependency pins.
Install NumPy/SciPy for analysis.

## Run and analyze

First derive each board's `master_trim` and main tuning sensitivity from its
own sweep. Add the trims to the local configuration. Set
`ACORN_MAIN_PPB_PER_CODE` and `SPEC_MAIN_PPB_PER_CODE` below to those slopes
in **ppb/code** (`wr_trim.py` reports ppm/code; multiply by 1000).

```sh
python3 tools/wr_jtag.py --config build/wr-bench.json start
python3 tools/wr_loop_response.py --config build/wr-bench.json --master spec --master-sensitivity-ppb "$SPEC_MAIN_PPB_PER_CODE" --output build/response-acorn
python3 tools/wr_loop_analysis.py build/response-acorn --sensitivity-ppb "$ACORN_MAIN_PPB_PER_CODE"
python3 tools/wr_loop_response.py --config build/wr-bench.json --master acorn --master-sensitivity-ppb "$ACORN_MAIN_PPB_PER_CODE" --output build/response-spec
python3 tools/wr_loop_analysis.py build/response-spec --sensitivity-ppb "$SPEC_MAIN_PPB_PER_CODE"
python3 tools/wr_qualify.py --config build/wr-bench.json --master-board acorn --duration 20 --output build/after-stimuli
python3 tools/wr_loop_response.py --config build/wr-bench.json --master acorn --passive 300 --output build/temperature-spec
python3 tools/wr_loop_analysis.py build/temperature-spec
python3 tools/wr_jtag.py --config build/wr-bench.json stop
```

Dynamic runs stop PTP, configure roles and apply phase/frequency stimuli at
half, normal and double PI gain. Cleanup restores saved PI gains, the requested
master trim and PTP. Run the qualifier afterward to check reacquisition.
Passive runs observe the existing roles. The capture tool holds the shared
bench lock and records UART/FIFO data, commands, sensors, diagnostics and input
hashes under `--output`. Keep these generated files outside version control.

The firmware's optional `pll step <absolute_ps>` command applies an atomic
phase step and emits 640 native main-loop samples before resuming decimation.
It requires stopped PTP and main lock, refuses overlapping bursts, and limits
each step to 500 ps. Drain the FIFO before injecting. Existing `pll sps` slews
the setpoint. Decimation affects logging only, not the controller update rate.
The runner excludes initial backlog after a ten-second warm-up and rejects
sample gaps or incomplete native captures.

## Interpret the output

Analysis fits a discrete PI controller, integrating oscillator and first-order
actuator lag. Reported loop margins depend on this model; bootstrap intervals
exclude model/systematic error. Internal phase traces do not establish physical
PPS alignment or jitter. XADC measures FPGA die temperature, not oscillator or
ambient temperature. Those measurements require external instruments/probes
and calibration with the intended final FPGA image.
