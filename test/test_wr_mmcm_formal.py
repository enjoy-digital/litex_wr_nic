#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Prove the phase-request protocol at the accepted-command boundary."""

import shutil
import subprocess
from pathlib import Path

import pytest

from migen import *

from litex.gen import LiteXModule
from litex.build.xilinx import XilinxPlatform

from litex_wr_nic.gateware.wr_clock import WRMMCMBackend

# Formal Protocol Tests ----------------------------------------------------------------------------

@pytest.mark.parametrize("width,div_n,timeout", [(16, 0, 1024), (8, 2, 32)])
def test_mmcm_protocol_for_arbitrary_accepted_commands(tmp_path, width, div_n, timeout):
    for tool in ("yosys", "sby", "boolector"):
        if shutil.which(tool) is None:
            pytest.skip(f"{tool} is required for the MMCM formal regression")

    dut = LiteXModule()
    dut.cd_sys = ClockDomain("sys")
    dut.backend = backend = WRMMCMBackend("sys", "sys",
        width=width, div_n=div_n, timeout_cycles=timeout)
    reset = Signal(name_override="core_reset")
    dut.comb += reset.eq(ResetSignal(backend.cdc.output_cd))
    ports = dict(
        psen          = backend.psen,
        psincdec      = backend.psincdec,
        psdone        = backend.psdone,
        busy          = backend.busy,
        fault         = backend.fault,
        steps         = backend.steps,
        accepted_data = backend.cdc.source.data,
        accepted_load = backend.cdc.source.load,
    )
    for name, signal in ports.items():
        signal.name_override = name
        signal.attr.add("keep")
    platform = XilinxPlatform("xc7a200tsbg484-3", [])
    platform.get_verilog(dut, name="wr_mmcm_dut",
        ios={dut.cd_sys.clk, dut.cd_sys.rst, reset, *ports.values()},
    ).write(str(tmp_path / "dut.v"))
    root = Path(__file__).resolve().parent
    (tmp_path / "formal.sv").write_text((root / "hdl/wr_mmcm_formal.sv").read_text()
        .replace("@WIDTH@", str(width)).replace("@TIMEOUT@", str(timeout)))

    # Cut only the accepted command wires: all values/load patterns are now
    # arbitrary, including patterns a paced FIFO would never produce. The
    # phase accumulator, request state, watchdog and reset logic are the real
    # generated RTL. CDC coherence itself is covered by separate RTL/hardware
    # tests; this proof does not model metastability or the analog MMCM.
    # Rate, neutral-code behavior and code-to-direction mapping are checked
    # by the independent command integral in the full-width XSim regression.
    (tmp_path / "mmcm.sby").write_text("""[tasks]
prove
cover

[options]
prove: mode prove
prove: depth 32
timeout 180
cover: mode cover
cover: depth 180

[engines]
prove: smtbmc boolector
cover: smtbmc boolector

[script]
read_verilog +/xilinx/cells_sim.v
read_verilog -formal dut.v formal.sv
prep -top wr_mmcm_formal -flatten
cutpoint w:dut.accepted_data w:dut.accepted_load
opt_clean -purge
async2sync
dffunmap

[files]
dut.v
formal.sv
""")
    result = subprocess.run(["sby", "-f", "mmcm.sby"],
        cwd            = tmp_path,
        capture_output = True,
        text           = True,
        timeout        = 240,
    )
    output = result.stdout + result.stderr
    (tmp_path / "formal-output.log").write_text(output)
    assert result.returncode == 0, output
    for task in ("prove", "cover"):
        assert (tmp_path / f"mmcm_{task}/status").read_text().split()[0] == "PASS", output
