#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Run real uRV instructions through the external-memory wrapper with XSim."""

import shutil
import subprocess
from pathlib import Path

import pytest

# Constants ----------------------------------------------------------------------------------------

ROOT = Path(__file__).resolve().parents[1]

# uRV External Memory Test -------------------------------------------------------------------------

def test_urv_external_memory_preserves_data_requests(tmp_path):
    for tool in ("xvlog", "xvhdl", "xelab", "xsim"):
        if shutil.which(tool) is None:
            pytest.skip(f"{tool} is required for the mixed-language uRV regression")
    cores = ROOT / "wr-cores"
    rtl   = cores / "ip_cores/urv-core/rtl"
    if not (rtl / "urv_cpu.v").exists():
        pytest.skip("The pinned wr-cores checkout is required")

    def run(tool, *args):
        result = subprocess.run([tool, *map(str, args)],
            cwd            = tmp_path,
            capture_output = True,
            text           = True,
            timeout        = 120,
        )
        output = result.stdout + result.stderr
        (tmp_path / f"{tool}-output.log").write_text(output)
        assert result.returncode == 0, output
        return output

    modules = (
        "cpu", "csr", "decode", "divide", "ecc", "exceptions", "exec",
        "fetch", "multiply", "regfile", "shifter", "timer", "writeback",
    )
    run("xvlog", "--work", "work", "--include", rtl,
        *[rtl / f"urv_{name}.v" for name in modules],
    )
    run("xvhdl", "--2008", "--work", "work",
        cores / "ip_cores/general-cores/modules/wishbone/wishbone_pkg.vhd",
        cores / "modules/wrc_core/wrc_cpu_csr.vhd",
        rtl / "urv_pkg.vhd",
        ROOT / "litex_wr_nic/gateware/wr-cores/modules/wrc_core/wrc_urv_external_memory.vhd",
        ROOT / "test/hdl/wr_urv_external_tb.vhd",
    )
    run("xelab", "work.urv_external_tb", "--snapshot", "urv_external_tb")
    output = run("xsim", "urv_external_tb", "--runall")
    # XSim may exit with status zero for a VHDL assertion failure.
    assert "Note: PASS" in output and "Failure:" not in output, output
