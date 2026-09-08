#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from tools.compare_wr_cpu_resources import parse_timing, parse_utilization
from tools.compare_wr_cpu_types import parse_intra_clock_wns

# Resource Report Tests ----------------------------------------------------------------------------

def test_parse_vivado_utilization(tmp_path):
    report = tmp_path / "utilization.rpt"
    report.write_text("""
| Tool Version : Vivado v.2025.2 (lin64) Build 1
| Device       : xc7a50tcsg325-2
| Slice LUTs                 | 100 | 0 | 0 | 1000 | 10.0 |
|   LUT as Logic             | 80  | 0 | 0 | 1000 | 8.0  |
|   LUT as Memory            | 20  | 0 | 0 | 1000 | 2.0  |
|     LUT as Distributed RAM | 18  | 0 |   |      |      |
|     LUT as Shift Register  | 2   | 0 |   |      |      |
| Slice Registers            | 120 | 0 | 0 | 2000 | 6.0  |
|   RAMB36/FIFO*             | 3   | 0 | 0 | 75   | 4.0  |
|   RAMB18                   | 5   | 0 | 0 | 150  | 3.3  |
| DSPs                       | 2   | 0 | 0 | 120  | 1.7  |
""", encoding="utf-8")
    values = parse_utilization(report)
    assert values["slice_luts"] == 100
    assert values["bram18_equivalent"] == 11
    assert values["device"] == "xc7a50tcsg325-2"


def test_parse_vivado_timing(tmp_path):
    report = tmp_path / "timing.rpt"
    report.write_text("""
WNS(ns) TNS(ns) TNS Failing Endpoints TNS Total Endpoints WHS(ns) THS(ns) THS Failing Endpoints THS Total Endpoints
------- ------- --------------------- ------------------- ------- ------- --------------------- -------------------
0.125 0.000 0 1200 0.050 0.000 0 1200
""", encoding="utf-8")
    values = parse_timing(report)
    assert values["wns_ns"] == 0.125
    assert values["setup_failing_endpoints"] == 0
    assert values["whs_ns"] == 0.050


def test_parse_wr_cpu_clock_timing(tmp_path):
    report = tmp_path / "timing.rpt"
    report.write_text("""
| Intra Clock Table
clk_sys 5.125 0.000 0 100 0.050 0.000 0 100
| Inter Clock Table
clk_sys other_clock -0.250 -1.000 4 100 0.050 0.000 0 100
""", encoding="utf-8")
    assert parse_intra_clock_wns(report, "clk_sys") == 5.125
