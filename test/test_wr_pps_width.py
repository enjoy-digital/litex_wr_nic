#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Check the portable pulse-width transformation applied to xwr_pps_gen."""

import re
import shutil
import subprocess
from pathlib import Path

import pytest

# Constants ----------------------------------------------------------------------------------------

ROOT      = Path(__file__).resolve().parents[1]
PPS_GEN   = ROOT / "wr-cores/modules/wr_pps_gen/xwr_pps_gen.vhd"
TESTBENCH = ROOT / "test/hdl/wr_pps_width_tb.vhd"

# PPS Width Tests ----------------------------------------------------------------------------------

def test_transformation_applies_to_the_pinned_source():
    if not PPS_GEN.exists():
        pytest.skip("The pinned wr-cores checkout is required")
    from litex_wr_nic.gateware.wr_phy import pps_width_replacements

    source = PPS_GEN.read_text()
    for before, _ in pps_width_replacements():
        assert source.count(before) == 1, before


def test_patched_pulse_width_matches_the_original(tmp_path):
    if shutil.which("ghdl") is None:
        pytest.skip("GHDL is required for the pulse-width equivalence check")
    # Both versions of the process run against the same stimulus; the
    # testbench fails on the first cycle where their outputs differ.
    result = subprocess.run(["ghdl", "-c", "--std=08", str(TESTBENCH), "-r", "wr_pps_width_tb"],
        cwd=tmp_path, capture_output=True, text=True, timeout=300)
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "PASS" in output and "failure" not in output.lower(), output
