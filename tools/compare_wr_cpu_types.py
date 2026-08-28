#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Compare placed SPEC-A7 utilization for uRV and LiteX WR CPUs."""

import csv
import json
import hashlib
import argparse
import subprocess
from pathlib import Path

from tools.compare_wr_cpu_resources import _delta, collect_mode, parse_hierarchy

# Constants ----------------------------------------------------------------------------------------

CONFIGS = (
    "urv-integrated",
    "vexriscv-lite-integrated",
    "urv-hyperram",
    "vexriscv-lite-hyperram",
)
PAIRS = {
    "integrated" : ("urv-integrated", "vexriscv-lite-integrated"),
    "hyperram"   :   ("urv-hyperram",   "vexriscv-lite-hyperram"),
}
DELTA_KEYS = (
    "slice_luts", "logic_luts", "lut_memory", "ffs", "ramb36", "ramb18",
    "bram18_equivalent", "dsps", "bit_bytes",
)

# Helpers ------------------------------------------------------------------------------------------

def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parse_intra_clock_wns(path, clock):
    """Return a clock's intra-domain WNS from a Vivado timing summary."""
    in_table = False
    for line in Path(path).read_text(encoding="utf-8", errors="replace").splitlines():
        if "| Intra Clock Table" in line:
            in_table = True
            continue
        if in_table and "| Inter Clock Table" in line:
            break
        fields = line.split()
        if in_table and fields and fields[0] == clock:
            return float(fields[1])
    raise ValueError(f"could not find intra-clock timing for {clock} in {path}")

# Comparison ---------------------------------------------------------------------------------------

def make_comparison(root, firmware_dir=None):
    root    = Path(root)
    configs = {}
    for name in CONFIGS:
        build_dir = root / name
        values    = collect_mode(build_dir)
        values["wr_clk_wns_ns"] = parse_intra_clock_wns(
            build_dir / values["reports"]["timing"], "clk_sys")
        values["hierarchy"] = parse_hierarchy(
            build_dir / values["reports"]["hierarchy"], (
                r"U_CPU_PRIVATE", r"U_CPU_EXTERNAL", r"VexRiscv",
                r"wrc_external_cpu_control", r"wr_cpu", r"hyperram",
                r"cache", r"boot",
            ))
        configs[name] = values
    reference = configs[CONFIGS[0]]
    for name, values in configs.items():
        if values["device"] != reference["device"]:
            raise ValueError(f"{name} used {values['device']}, expected {reference['device']}")
        if values["tool_version"] != reference["tool_version"]:
            raise ValueError(
                f"{name} used {values['tool_version']}, expected {reference['tool_version']}")

    deltas = {}
    for memory, (urv_name, vex_name) in PAIRS.items():
        urv            = configs[urv_name]
        vex            = configs[vex_name]
        deltas[memory] = {key: _delta(vex[key], urv[key]) for key in DELTA_KEYS}
        deltas[memory].update({
            "wns_ns"        :        vex["wns_ns"] - urv["wns_ns"],
            "whs_ns"        :        vex["whs_ns"] - urv["whs_ns"],
            "wr_clk_wns_ns" : vex["wr_clk_wns_ns"] - urv["wr_clk_wns_ns"],
        })

    try:
        git_sha = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True,
            stderr=subprocess.DEVNULL).strip()
        git_dirty = bool(subprocess.check_output(
            ["git", "status", "--porcelain"], text=True,
            stderr=subprocess.DEVNULL).strip())
    except (OSError, subprocess.CalledProcessError):
        git_sha   = "unknown"
        git_dirty = None

    firmware_hashes = {}
    if firmware_dir is not None:
        firmware_dir = Path(firmware_dir)
        for cpu_type, filename in (
            ("urv", "spec_a7_wrc.bin"),
            ("vexriscv-lite", "spec_a7_wrc_vexriscv.bin"),
        ):
            firmware_hashes[cpu_type] = _sha256(firmware_dir / filename)

    return {
        "metadata": {
            "git_sha"         : git_sha,
            "git_dirty"       : git_dirty,
            "target"          : "spec_a7_wr_nic",
            "device"          : reference["device"],
            "tool_version"    : reference["tool_version"],
            "firmware_sha256" : firmware_hashes,
            "wr_cpu_clk_hz"   : 62_500_000,
            "directives"      : {
                "place"               : "Explore",
                "route"               : "Explore",
                "post_route_phys_opt" : "Explore",
            },
        },
        "configs"               : configs,
        "vexriscv_delta_vs_urv" : deltas,
    }

# Report Output ------------------------------------------------------------------------------------

def _format_delta(delta):
    text = f"{delta['absolute']:+g}"
    if delta["percent"] is not None:
        text += f" ({delta['percent']:+.1f}%)"
    return text


def write_markdown(comparison, path):
    memory_labels = {"integrated": "Integrated", "hyperram": "HyperRAM"}
    metrics = (
        ("Slice LUTs", "slice_luts"),
        ("Logic LUTs", "logic_luts"),
        ("LUT memory", "lut_memory"),
        ("Flip-flops", "ffs"),
        ("RAMB36", "ramb36"),
        ("RAMB18", "ramb18"),
        ("BRAM18 equivalents", "bram18_equivalent"),
        ("DSPs", "dsps"),
        ("Bitstream bytes", "bit_bytes"),
        ("WR clock WNS (ns)", "wr_clk_wns_ns"),
        ("WNS (ns)", "wns_ns"),
        ("WHS (ns)", "whs_ns"),
        ("Build time (s)", "build_seconds"),
    )
    meta = comparison["metadata"]
    lines = [
        "# WR CPU Type Resource Comparison", "",
        f"- Target: `{meta['target']}` / `{meta['device']}`",
        f"- Tool: `{meta['tool_version']}`",
        f"- Git: `{meta['git_sha']}`{' (dirty)' if meta['git_dirty'] else ''}",
        f"- CPU clock: `{meta['wr_cpu_clk_hz']/1e6:g} MHz`", "",
    ]
    for memory, (urv_name, vex_name) in PAIRS.items():
        urv   = comparison["configs"][urv_name]
        vex   = comparison["configs"][vex_name]
        delta = comparison["vexriscv_delta_vs_urv"][memory]
        lines += [
            f"## {memory_labels[memory]} memory", "",
            "| Metric | uRV | VexRiscv lite | VexRiscv Δ |",
            "|---|---:|---:|---:|",
        ]
        for label, key in metrics:
            if key in DELTA_KEYS:
                delta_text = _format_delta(delta[key])
            elif key in ("wns_ns", "whs_ns", "wr_clk_wns_ns"):
                delta_text = f"{delta[key]:+.3f}"
            elif urv[key] is not None and vex[key] is not None:
                delta_text = f"{vex[key] - urv[key]:+.3f}"
            else:
                delta_text = "n/a"
            lines.append(f"| {label} | {urv[key]} | {vex[key]} | {delta_text} |")
        lines += [
            "",
            f"Resource result: VexRiscv changes Slice LUTs by "
            f"{_format_delta(delta['slice_luts'])}, DSPs by "
            f"{_format_delta(delta['dsps'])}, and BRAM18 equivalents by "
            f"{_format_delta(delta['bram18_equivalent'])}.", "",
            f"Timing: uRV {'MET' if urv['wns_ns'] >= 0 else 'VIOLATED'} "
            f"({urv['wns_ns']:+.3f} ns WNS), VexRiscv "
            f"{'MET' if vex['wns_ns'] >= 0 else 'VIOLATED'} "
            f"({vex['wns_ns']:+.3f} ns WNS).", "",
            f"WR-clock timing: uRV {'MET' if urv['wr_clk_wns_ns'] >= 0 else 'VIOLATED'} "
            f"({urv['wr_clk_wns_ns']:+.3f} ns WNS), VexRiscv "
            f"{'MET' if vex['wr_clk_wns_ns'] >= 0 else 'VIOLATED'} "
            f"({vex['wr_clk_wns_ns']:+.3f} ns WNS).", "",
        ]
    lines += [
        "All utilization figures are from placed designs; timing is from the final timing report.", "",
    ]
    Path(path).write_text("\n".join(lines), encoding="utf-8")


def write_csv(comparison, path):
    keys = (
        "slice_luts", "logic_luts", "lut_memory", "ffs", "ramb36", "ramb18",
        "bram18_equivalent", "dsps", "wns_ns", "tns_ns", "whs_ns", "ths_ns",
        "wr_clk_wns_ns", "bit_bytes", "build_seconds",
    )
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=("config",) + keys, lineterminator="\n")
        writer.writeheader()
        for name in CONFIGS:
            writer.writerow({
                "config": name,
                **{key: comparison["configs"][name].get(key) for key in keys},
            })

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", help="Directory containing the four build directories.")
    parser.add_argument("--firmware-dir")
    parser.add_argument("--markdown", default="doc/wr_cpu_type_resource_comparison.md")
    parser.add_argument("--json", default="doc/wr_cpu_type_resource_comparison.json")
    parser.add_argument("--csv", default="doc/wr_cpu_type_resource_comparison.csv")
    args = parser.parse_args()

    comparison = make_comparison(args.root, firmware_dir=args.firmware_dir)
    for path in (args.markdown, args.json, args.csv):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
    write_markdown(comparison, args.markdown)
    Path(args.json).write_text(json.dumps(comparison, indent=2) + "\n", encoding="utf-8")
    write_csv(comparison, args.csv)


if __name__ == "__main__":
    main()
