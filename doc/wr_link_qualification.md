# Acorn / SPEC-A7 White Rabbit link qualification

This procedure uses the physical WR UART on each board and two independent
JTAGBone servers. The host needs only USB connections. The WR link itself runs
between Acorn SFP0 and **SPEC-A7 J12, which is SFP0**.

The [complete upstream hardware results](wr_link/results-upstream/README.md) include both
30-minute role tests, all twelve recovery cycles, raw UART/snapshot evidence,
image and firmware hashes, and the failed runs that led to the fixes. The
[upstream build notes](wr_link/upstream-build.md) identify the official source
revisions and integration changes. The [earlier results](wr_link/results/README.md)
used older project pins and remain available as historical evidence.

The implementation starts from `main` at
`dcb63a88a666f6aa66f0d4618d03d947abc0c25a`, with uRV and private WR CPU RAM.
The findings below concern this configuration. They do not qualify PCIe traffic,
other CPU/memory profiles, external-reference grandmaster operation, or absolute
PPS accuracy.

## Reproduced faults and fixes

| Symptom | Cause | Change / evidence |
| --- | --- | --- |
| Acorn bitstream generation fails with Vivado AVAL-139 | Fractional MMCM dividers were combined with fine phase shifting. | Use integer dividers and the PSDONE-aware backend from the earlier [clock work](https://github.com/enjoy-digital/litex_wr_nic/pull/77). |
| Acorn clock commands can lose small corrections or overlap phase shifts | Commands cross clock domains and phase shifts need completion handshakes. | Transfer coherent 16-bit commands through a FIFO, preserve fractional phase across same-direction updates, wait for PSDONE, expose a completion watchdog and counters. |
| Acorn slave periodically leaves `TRACK_PHASE` while the link and PLL remain locked | The original MMCM PI profile (`kp=-150, ki=-2`) produces phase excursions past PPSI's ±120 ps correction threshold on this bench. | Increase main-loop tracking gains to `kp=-600, ki=-16`; reduce the frequency prelock boost from 20 to 5 to preserve its proportional gain. See the measured comparison below. |
| Acorn timing fails on reset crossings and PCIe clock-mode paths | The clock-generator reset crossed into 200 MHz unsynchronized; PIPE clock routing/constraints included incompatible clock modes. | Synchronize reset, buffer PSCLK, use dedicated PIPE MMCM outputs and constrain the mutually exclusive mux inputs. |
| SPEC timing fails on PCIe clock-mode paths | Paths between mutually exclusive PIPE clocks were timed together. | Port the focused clock-mode constraint from [PR 78](https://github.com/enjoy-digital/litex_wr_nic/pull/78). |
| Disabling the external WR clock still elaborates external-clock logic | A quoted `"FALSE"` was passed to a VHDL boolean through mixed-language elaboration. | Pass a numeric boolean, as for the CPU generics. |
| Rebuilding the pinned firmware retains a different PPSI version | Checking out WRPC did not synchronize its submodules. | Run `git submodule update --init --recursive`; test a reused checkout across two pinned submodule revisions. |
| SPI flash identification reads all ones | WR-core cleared MOSI on a GPSR write instead of the GPCR write strobe. | [Upstream ff0d6950](https://gitlab.com/ohwr/project/wr-cores/-/commit/ff0d6950) is included in the current WR-core pin; GHDL verifies it and reproduces the original typo as a negative control. |
| Host diagnostic RAM lacks version 2 and live data | Firmware used CPU offset `0x900`; the CPU map places diagnostics at `0x800`. | Correct `BASE_WDIAGS_PRIV`. The **host** diagnostic offset remains `0x900`. |
| Snapshot requests are ignored | The WR-core host write selector enabled word 0 (VER), while v2 CTRL is word 1. | Allow writes to CTRL at byte offset `0x04`, leaving VER/data read-only. Test the actual decoder in GHDL. |
| An SFP storage error is reported as a calibration match | `storage_match_sfp()` negative errno values passed the `== 0` check. | Treat all non-positive results as unmatched; propagate negative errors. Test the actual C function. |
| Two standard JTAG servers collide | The CLI could not select JTAGUART's internal stream port. | Add `--jtag-port` in [LiteX PR 2590](https://github.com/enjoy-digital/litex/pull/2590). Each board needs a distinct public and internal port. |
| Full upstream SPEC build misses timing in the 250 MHz PTM sniffer | Packet classification unnecessarily gated the FIFO data path in addition to its write enable. | Drive FIFO data directly in [LitePCIe PR 186](https://github.com/enjoy-digital/litepcie/pull/186), with unchanged valid/ready behavior and latency; use the SPEC target's improved placement settings. |
| Full upstream firmware reports `PRINTF OVF` and corrupts console fields | The project allocated only 16 bytes for an unbounded formatter; older firmware silently overflowed the same buffer. | Allocate 256 bytes, matching upstream SPEC defaults; reproduce the old overflow with AddressSanitizer, check the compiled size, and reject warnings during qualification. |

The SPI fix restores SPEC's original flash-backed MAC, `08:87:40:00:00:01`.
Acorn's flash identifies correctly but has no SDB filesystem at the probed
locations, so its firmware uses the fallback MAC. These two boards have distinct
MACs; a bench with two fallback addresses must set distinct **volatile** MACs
with `mac set`, or peer selection is ambiguous.

## Build reproducibly without changing persistent storage

Build each board in a separate checkout, or build sequentially and archive each
board's firmware and CSR map before building the next. Both targets generate
`test/csr.csv`, and both firmware builds use `spec_a7_wrc.*` filenames even for
Acorn. Acorn also changes the software PLL gains, so sharing a live firmware
checkout between simultaneous builds is incorrect.

```sh
python3 acorn_wr_nic.py --build --wr-cpu-type urv --wr-cpu-memory private \
    --wr-read-only-storage --output-dir build/acorn-wr
cp test/csr.csv build/acorn-wr/csr.csv
cp litex_wr_nic/firmware/spec_a7_wrc.* build/acorn-wr/
```

In the SPEC checkout:

```sh
python3 spec_a7_wr_nic.py --build --wr-cpu-type urv --wr-cpu-memory private \
    --wr-read-only-storage --output-dir build/spec-wr
cp test/csr.csv build/spec-wr/csr.csv
cp litex_wr_nic/firmware/spec_a7_wrc.* build/spec-wr/
```

`--wr-read-only-storage` is a **firmware build option**. It keeps automatic RX
timestamp calibration in RAM and prevents SPI flash writes/erases, including
writes requested by a stored startup script. A console message saying calibration
was saved is therefore a RAM-only result in this build. The option cannot make a
previously built image read-only. The ordinary firmware build retains its usual
persistent storage behavior.

The qualified toolchain is Vivado 2024.1 and the project's `riscv-11.2-small`
toolchain. The firmware pins official WRPC
`13527cd68e1833214a89e4ee8c5b208188ff0e6a`, which pins PPSI
`33d8c46c8353be56d35dc57dd3492769e276c00f`. WR-core is built from official
`8cc5e53275d229fbbf50b96a6ae4cb9c87626d53`, with the project integration patches
applied. Both recursive submodule trees are initialized at their upstream pins.
Use LitePCIe `f9a2d43e4e9c29ae837641fb1c86cc0ffdec5da4` from PR 186 for these
builds, selecting that checkout with `PYTHONPATH` if necessary. Existing
WR-core checkouts at a different revision are rejected; move them aside before
building. See the [complete source notes](wr_link/upstream-build.md).

Review the final timing report before loading. The tested clean builds meet the
specified constraints:

| Board | Setup WNS | Hold WHS | Pulse-width slack |
| --- | ---: | ---: | ---: |
| Acorn | +0.174 ns | +0.036 ns | 0 ns |
| SPEC-A7 | +0.045 ns | +0.057 ns | 0 ns |

Neither has unclocked registers or unconstrained active internal endpoints.
Acorn reports two constant-clock endpoints in the measurement channels for its
unconnected external clocks. The existing asynchronous/slow external I/O paths
have no input/output delay specifications. These reports do not constitute
external PPS accuracy or PCIe hardware qualification.

## Identify and load the USB devices

The two FT4232H adapters on this bench have no serial numbers. Their shared
`/dev/serial/by-id` name cannot identify a board. Use physical USB locations for
JTAG and `/dev/serial/by-path` for UART; verify the pairing before programming.
The observed mappings were:

| Board | USB location | UART path | Public JTAG port | Internal stream port |
| --- | --- | --- | ---: | ---: |
| Acorn | `5-2` | `/dev/serial/by-path/pci-0000:0b:00.3-usb-0:2:1.2-port0` | 1236 | 20002 |
| SPEC | `1-3` | `/dev/serial/by-path/pci-0000:06:00.1-usb-0:3:1.2-port0` | 1235 | 20001 |

Bus/device numbers can change on reconnect. Read `busnum` and `devnum` under
`/sys/bus/usb/devices/<location>/` immediately before using openFPGALoader.

The Acorn device identifies as a 200T. The SPEC device identifies as a 35T while
the existing project target generates a 50T image and converts its ID/CRC. Follow
the project's existing conversion flow and load the resulting SPEC **`.bin`**:

```sh
python3 litex_wr_nic/gateware/xilinx-bitstream.py \
    build/spec-wr/gateware/spec_a7_wr_nic.bit \
    build/spec-wr/gateware/spec_a7_wr_nic.bin
```

Stop that board's JTAG server before loading. Use explicit USB selection and
SRAM programming, for example with the bus/device numbers observed on this bench:

```sh
openFPGALoader --cable ft4232 --busdev-num 5:6 --freq 5000000 \
    --bitstream build/acorn-wr/gateware/sqrl_acorn.bit
openFPGALoader --cable ft4232 --busdev-num 1:17 --freq 5000000 \
    --bitstream build/spec-wr/gateware/spec_a7_wr_nic.bin
```

`tools/wr_link.py console --load-bitstream ... --usb-busdev ...` performs the
same SRAM load while recording physical UART boot output and the image hash.
It does not stop an existing JTAG server. Its `passed` field means the requested
capture/commands completed; use the qualification tool for a WR pass/fail result.

## Run two JTAGBone servers

Use LiteX with PR 2590, or a version containing it. Create one OpenOCD file per
board with the appropriate USB location:

```tcl
adapter driver ftdi
adapter speed 5000
adapter usb location 1-3
transport select jtag
ftdi_vid_pid 0x0403 0x6011
ftdi_channel 0
ftdi_layout_init 0x00e8 0x60eb
reset_config none
tcl_port disabled
telnet_port disabled
gdb_port disabled
source [find cpld/xilinx-xc7.cfg]
```

For Acorn change the location to `5-2`. Start the servers in separate terminals:

```sh
litex_server --jtag --jtag-config spec-usb.cfg --jtag-port 20001 \
    --bind-ip 127.0.0.1 --bind-port 1235
litex_server --jtag --jtag-config acorn-usb.cfg --jtag-port 20002 \
    --bind-ip 127.0.0.1 --bind-port 1236
```

Always use each image's own CSR map. Check the LiteX identifier and WR map magic
`0x57525043` before interpreting registers. The qualification tools do both.
The physical UARTs run at 115200 baud and remain separate from JTAGBone.

## Set roles and qualify real WR synchronization

Acorn as master requires a volatile frequency trim on this particular pair:

```text
ptp stop
mode master
pll sdac 0 35000
ptp start
```

Set SPEC with `ptp stop`, `mode slave`, `ptp start`. The valid command is
`mode master` or `ptp master`; `ptp mode master` is not accepted.

With Acorn at its nominal midscale code 32768, SPEC's main PLL reached its low
rail and WR link setup timed out in `WR_S_LOCK`, eventually falling back to
ordinary PTP. Code 35000 brings the frequency into SPEC's tuning range and the
slave reaches WR `TRACK_PHASE`. This is a bench frequency adjustment, not SFP
delay calibration or a universal default for all Acorns.

The generated Acorn MMCM uses a 1.5 GHz VCO and a 200 MHz PSCLK. Its backend
requests `(code - 32768) / 2**19` signed phase steps per PSCLK. With the
[7-series fine phase step](https://docs.amd.com/v/u/en-US/ug472_7Series_Clocking)
of one fifty-sixth of a VCO period, this corresponds to approximately
0.00454 ppm per code. Thus 35000 adds about 10.14 ppm relative to midscale.
This is a calculation from the implemented tuning law, not a measurement of
absolute oscillator accuracy. Host wall-clock estimates from sequential JTAG
counter latches are too noisy to establish that accuracy.

For the reverse direction, use SPEC `mode master` at its default tuning code
and Acorn `mode slave`. Initial functional tests reached WR `TRACK_PHASE` in
both directions. The completed qualification results are recorded separately.

### Acorn slave PLL tuning

The first reverse soak failed after 46 seconds of tracking: the reported offset
reached −125 ps, so PPSI changed from `TRACK_PHASE` to `SYNC_PHASE`. This is the
existing `wrh-servo.c` safeguard at twice the 60 ps stability threshold. The link,
WR extension, frequency lock, and main PLL remained active; the servo recovered
without intervention. MMCM completion-fault and superseded-command counters were
zero. A further baseline observation reproduced the excursions.

The following UART gain changes were compared on the same running image, with
SPEC master and Acorn slave. Statistics exclude the first 30 seconds of each
observation. These are internal servo estimates, not independent PPS measurements.

| Acorn main PI (`kp`, `ki`, shift) | Observation | Offset range | Standard deviation |
| --- | ---: | ---: | ---: |
| −150, −2, 12 (old default) | 180 s | −124…+100 ps | 49.66 ps |
| −150, −1, 12 | Stopped at 104 s after larger excursions | −291…+229 ps | 99.38 ps |
| −300, −8, 12 | 180 s | −31…+30 ps | 12.33 ps |
| −600, −16, 12 | 180 s | −20…+14 ps | 6.14 ps |

Both stronger profiles retained tracking after the initial monitor setup. The
selected profile is `−600/−16`, with the helper PLL unchanged. The firmware also
reduces `MPLL_FREQ_PRELOCK_GAIN_BOOST` to 5: `600 × 5 = 150 × 20`, preserving the
old acquisition proportional gain instead of multiplying it by four. Warm UART
tuning alone does not test this acquisition path; it must also pass the SRAM
reload and PCS recovery tests with the rebuilt firmware. The qualification
tool and PPSI thresholds are unchanged.

Run the observer/configurator as follows, using the UART paths from the table:

```sh
python3 tools/wr_qualify.py \
    --master-uart "$ACORN_UART" --master-jtag 1236 --master-csr build/acorn-wr/csr.csv \
    --slave-uart "$SPEC_UART" --slave-jtag 1235 --slave-csr build/spec-wr/csr.csv \
    --configure --master-trim 35000 --duration 1800 --acquire-timeout 300 \
    --output build/qualification/acorn-master
```

For SPEC as master, exchange the complete master/slave argument groups and omit
`--master-trim`. Omit `--configure` when observing an already configured pair.
The output directory must be new, preserving earlier failed attempts.

The test requires both physical UART monitors to show the intended PTP roles,
matching distinct peer MACs, frequency lock, and `IDLE / EXT_ON`. It independently
requires the slave's `White-Rabbit: TRACK_PHASE`, diagnostic servo state 4, link
and PLL lock, fresh advancing time and servo updates, and unchanged RX error
counts. Every complete UART frame is checked during the soak. A loss fails the
run; the timer is not silently restarted.

The diagnostic `WR_MODE` bit alone is insufficient: this firmware populates it
from servo validity, including plain PTP. A valid-bit-only test would accept the
very fallback that initially hid the WR failure.

Outputs include full UART byte streams, transmitted commands/responses, decoded
GUI frames, coherent diagnostic snapshots, and a machine-readable summary.
Offsets are the servo's own estimates. Free-running master time initially starts
at the firmware epoch; this tests transfer and advancement of that time scale.
It does not establish traceability to UTC/TAI.

## Recovery tests

`tools/wr_recover.py` uses a JSON file containing two named board records, each
with `uart`, `usb_location`, `jtag_port`, `stream_port`, `jtag_config`, `csr`, and
`bitstream`. Start from [bench.example.json](wr_link/bench.example.json) and the
accompanying OpenOCD files, adapting the USB/UART locations to your bench.
Run from the repository root; paths refer to the existing qualified images. Stop existing debug
servers first: the tool owns and cleans up only the process groups it starts.

```sh
python3 tools/wr_recover.py --config bench.json --master acorn --master-trim 35000 \
    --operation both --cycles 3 --output build/qualification/acorn-recovery
```

The link test sets the slave PCS's MDIO power-down bit for at least five seconds,
records link-down on **both** boards, restores the original MCR even on a test
failure, and observes automatic WR reacquisition. It does not manually restart
PTP during recovery. This exercises SERDES/PCS interruption; it does not simulate
module removal or test connector contacts.

The host endpoint window contains only MAC registers. MDIO is accessed through
the existing **uRV debug instruction interface**, using CPU address `0x100100`.
The helper briefly pauses the slave CPU around each control transaction, saves
and restores the registers it uses, and resumes execution before observing the
link. Its instruction sequence is tested against the real uRV RTL in XSim.
The physical UART and JTAG diagnostics are exercised while the PCS is held down.
This recovery helper is for uRV images and must not be used during an uninterrupted
timing observation because the software PLL pauses during CPU debug access.

WRPC prints diagnostics asynchronously and performs blocking PHY/SFP work when
the link returns. Console capture preserves those messages but excludes
timestamped PPSI diagnostics when recognizing an interleaved command echo.
Recovery leaves verbose diagnostics disabled and lets the following qualifier
connect to the shell before sending further queries. A command sent immediately
after CPU resume can be dropped while link initialization is still running.

The reload test re-identifies USB devices, checks that UART/JTAG share the same
physical adapter, SRAM-loads both images with UART boot capture, restarts debug
servers, configures the intended roles, and verifies reacquisition. Recovery must
complete within five minutes and remain tracking for the requested observation
period. Repeat with `--master spec` and no master trim.

## Calibration and remaining physical checks

SPEC reads a Tyco `2127931-2` module/cable identity but has no matching calibration
record. Acorn's EEPROM reads return all ones and fail checksum. The data link
nevertheless runs WR. The Acorn baseboard's SFP I2C needs the jumper routing
described in the original [bring-up issue](https://github.com/enjoy-digital/litex_wr_nic/issues/2);
the present jumper placement has not been physically confirmed. An all-ones
read alone does not prove which part of that connection is absent.

No SFP delay/asymmetry constants were invented or written to flash. Automatic
PHY/RX timestamp calibration runs in RAM. Absolute timing accuracy needs the
appropriate physical calibration and an independent PPS measurement.

The qualified SPEC image retains the legacy AD5683R driver's effective x2 gain
on both DACs. Its misspelled VHDL generic is ignored, so the VHDL default applies
even where the Python DMTD argument says `gain=1`. The driver correction already
exists in [PR 77](https://github.com/enjoy-digital/litex_wr_nic/pull/77). This
qualification does not change that analog range.

The legacy SPEC `*_dac_current` CSRs also follow the WR write-data bus every
clock, rather than latching only the corresponding DAC load strobe. That bus is
shared by the main/helper DAC writes and other SoftPLL registers. A snapshot can
therefore show zero or another register's data instead of the last DAC command.
Use UART `pll stat` / `pll gdac` to inspect the controller's commands; these are
still command values, not measured analog voltages. The loaded-image raw CSR
capture is retained with the final UART evidence so this difference is visible.

The existing firmware init string also contains `vlan off` while VLAN commands
are disabled. Its reported unknown-command warning does not stop later role/PTP
startup commands; VLAN handling in the endpoint is already disabled.

## Upstream submissions and method

[LiteX PR 2590](https://github.com/enjoy-digital/litex/pull/2590) contains the
independent JTAG stream-port fix. [LitePCIe PR 186](https://github.com/enjoy-digital/litepcie/pull/186)
contains the PTM formatter timing fix. The SPI MOSI fix already exists in WR-core.
The remaining focused WR-core/WRPC fixes have public PRs in unofficial GitHub
contribution mirrors, each based directly on a recorded official upstream revision:
[WR-core #1](https://github.com/enjoy-digital/wr-cores/pull/1),
[WRPC #1](https://github.com/enjoy-digital/wrpc-sw/pull/1), and
[WRPC #2](https://github.com/enjoy-digital/wrpc-sw/pull/2).
[wr_link/upstream](wr_link/upstream/README.md) contains the patches, upstream
migration findings and fresh regression results. The official repositories are
available over HTTPS at `gitlab.com/ohwr/project/`; GitLab SSH authentication is
unnecessary to fetch and rebuild them. PRs were created only in `enjoy-digital`
repositories. No merge requests were submitted to third-party repositories.

The workflow follows the project's [FPGA development article](https://enjoy-digital.github.io/posts/ai-era-fpga/)
and [M2SDR debugging guide](https://github.com/enjoy-digital/litex_m2sdr/blob/main/doc/debugging-guide.md):
preserve the starting tree, reproduce on hardware, verify the access path,
inspect generated logic, add focused observability and regressions, then record
what the hardware actually demonstrates.
