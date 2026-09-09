# Managing White Rabbit

Install the package with `pip install -e .` to get `litex_wr`, or run
`python3 -m litex_wr_nic.wr` directly from this checkout. Start `litex_server`
with your usual JTAGBone, UARTBone, Etherbone or PCIe transport, and use the
`csr.csv` generated with the **loaded bitstream**:

```sh
python3 -m litex_wr_nic.wr --csr-csv test/csr.csv status
python3 -m litex_wr_nic.wr --csr-csv test/csr.csv console
python3 -m litex_wr_nic.wr --csr-csv test/csr.csv console --command ver
python3 -m litex_wr_nic.wr --csr-csv test/csr.csv diagnose
python3 -m litex_wr_nic.wr --csr-csv test/csr.csv restart
python3 -m litex_wr_nic.wr --csr-csv test/csr.csv load-firmware \
    litex_wr_nic/firmware/spec_a7_wrc_vexriscv.bin --firmware-cpu vexriscv
```

Use `--host` and `--port` for a remote server. Transport selection belongs to
`litex_server`; the management commands use the same interface for all of them.
`--wr-region`, `--memory-region` and `--uart-prefix` support custom integration
names. The default regions are `wr_wb_slave` and `wr_cpu_mem`.

The console selects UARTShared's crossover port while it is open and restores
the previous selection on exit. Exit with **Ctrl-]**. Scripted commands wait
for `wrc#`; input is paced for the WRPC polling UART. A separate UARTBone path
must be used if the physical WR console and bridge would share the same pins.
UARTShared now defaults to a 4096-byte receive FIFO (plus its output buffer),
reports its occupancy for fixed-address burst reads and detects overflow.
Older 128-byte FIFO images can lose long replies over slow transports; use
short commands such as `uptime`/`time` or load an updated image.

`status` reads the live WR configuration signature, CPU selection, memory mode,
memory readiness, link/time-valid flags and host reset diagnostics. It also
prints available boot-loader and memory-bridge diagnostic registers. The
SHA-256 identifies the BRAM firmware file supplied **at FPGA build time**;
it does not claim to identify a later host-loaded image. Reset reason/count
cover system reset and writes to the WR host CPU reset register, not a
firmware watchdog. `diagnose` additionally exercises `ver`, `uptime` and
`time` through the console. An operational console does not establish WR
servo lock or PPS accuracy.

Older #67/#68 bitstreams have no live information bank. Private memory implies
uRV; for external memory supply `--cpu-type urv` or `--cpu-type vexriscv` before
the command. New bitstreams reject a CPU selection that conflicts with the
live configuration.

Firmware upload checks the declared CPU profile and size before stopping the
CPU, zero-fills the reserved memory, writes the image, verifies every word,
then releases reset. A failed verification leaves the CPU held in reset.
Private uRV uses the WR host memory window's big-endian word convention;
SoC memory uses little-endian words. Host writes use the shared memory path,
including the HyperRAM cache; do not write directly to its backing storage.
The VexRiscv instruction cache is reset with the CPU before a reload.

For uRV, `restart` and upload request a debug halt and wait for acknowledgement
before asserting reset. This avoids the intermittent early direct-reset hang
also observed on the upstream private-memory implementation. A halt timeout
restores the previous debug request without asserting reset. Recovery of an
already hung CPU can still require FPGA reload. The underlying direct-reset
failure is under investigation; the tool implements the tested workaround.
