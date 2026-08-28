#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Compare placed SPEC-A7 utilization for WR CPU memory implementations."""

import re
import csv
import json
import hashlib
import argparse
import subprocess
from pathlib import Path

# Constants ----------------------------------------------------------------------------------------

MODES = ("private", "integrated", "hyperram")
UTILIZATION_KEYS = {
    "Slice LUTs"             : "slice_luts",
    "LUT as Logic"           : "logic_luts",
    "LUT as Memory"          : "lut_memory",
    "LUT as Distributed RAM" : "lutram",
    "LUT as Shift Register"  : "srls",
    "Slice Registers"        : "ffs",
    "RAMB36/FIFO*"           : "ramb36",
    "RAMB18"                 : "ramb18",
    "DSPs"                   : "dsps",
}

# Report Parsing -----------------------------------------------------------------------------------

def _find_report(build_dir, suffix):
    matches = sorted((Path(build_dir) / "gateware").glob(f"*{suffix}"))
    if len(matches) != 1:
        raise FileNotFoundError(
            f"expected one *{suffix} below {build_dir}/gateware, found {len(matches)}")
    return matches[0]


def _row_fields(line):
    return [field.strip() for field in line.strip().strip("|").split("|")]


def parse_utilization(path):
    text   = Path(path).read_text(encoding="utf-8", errors="replace")
    result = {}
    for line in text.splitlines():
        fields = _row_fields(line) if line.lstrip().startswith("|") else []
        if len(fields) < 2:
            continue
        # Synthesis reports spell this row ``Slice LUTs*`` while placed
        # reports omit the footnote marker. Keep the meaningful star in the
        # separate ``RAMB36/FIFO*`` label.
        label = "Slice LUTs" if fields[0] == "Slice LUTs*" else fields[0]
        if label in UTILIZATION_KEYS and UTILIZATION_KEYS[label] not in result:
            try:
                result[UTILIZATION_KEYS[label]] = int(fields[1].replace(",", ""))
            except ValueError:
                pass
    missing = sorted(set(UTILIZATION_KEYS.values()) - set(result))
    if missing:
        raise ValueError(f"missing utilization fields in {path}: {', '.join(missing)}")
    result["bram18_equivalent"] = 2*result["ramb36"] + result["ramb18"]
    tool                        = re.search(r"\| Tool Version\s*:\s*(.+)", text)
    device                      = re.search(r"\| Device\s*:\s*(\S+)", text)
    result["tool_version"]      = tool.group(1).strip() if tool else "unknown"
    result["device"]            = device.group(1) if device else "unknown"
    return result


def parse_timing(path):
    lines = Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
    for n, line in enumerate(lines):
        if "WNS(ns)" not in line or "TNS(ns)" not in line or "WHS(ns)" not in line:
            continue
        for candidate in lines[n + 1:n + 5]:
            values = re.findall(r"-?\d+(?:\.\d+)?", candidate)
            if len(values) >= 8:
                return {
                    "wns_ns"                  : float(values[0]),
                    "tns_ns"                  : float(values[1]),
                    "setup_failing_endpoints" : int(values[2]),
                    "whs_ns"                  : float(values[4]),
                    "ths_ns"                  : float(values[5]),
                    "hold_failing_endpoints"  : int(values[6]),
                }
    raise ValueError(f"could not find the design timing summary in {path}")


def parse_hierarchy(path, patterns):
    rows = []
    for line in Path(path).read_text(encoding="utf-8", errors="replace").splitlines():
        if not line.startswith("|"):
            continue
        fields = _row_fields(line)
        if len(fields) != 10 or fields[0] == "Instance":
            continue
        instance, module = fields[:2]
        if not any(re.search(pattern, f"{instance} {module}", re.IGNORECASE) for pattern in patterns):
            continue
        try:
            values = [int(value) for value in fields[2:]]
        except ValueError:
            continue
        rows.append({
            "instance" : instance,
            "module"   : module,
            **dict(zip(("total_luts", "logic_luts", "lutram", "srls", "ffs",
                "ramb36", "ramb18", "dsps"), values)),
        })
    return rows


def collect_mode(build_dir):
    build_dir        = Path(build_dir)
    utilization_path = _find_report(build_dir, "_utilization_place.rpt")
    timing_path      = _find_report(build_dir, "_timing.rpt")
    hierarchy_path   = _find_report(build_dir, "_utilization_hierarchical_place.rpt")
    result           = parse_utilization(utilization_path)
    result.update(parse_timing(timing_path))
    result["hierarchy"] = parse_hierarchy(hierarchy_path, (
        r"U_CPU_PRIVATE", r"U_CPU_EXTERNAL", r"wr_cpu", r"hyperram", r"cache", r"boot"))
    result["reports"] = {
        "utilization" : str(utilization_path.relative_to(build_dir)),
        "timing"      : str(timing_path.relative_to(build_dir)),
        "hierarchy"   : str(hierarchy_path.relative_to(build_dir)),
    }
    gateware_dir = Path(build_dir) / "gateware"
    for extension in ("bit", "bin"):
        images                       = sorted(gateware_dir.glob(f"*.{extension}"))
        result[f"{extension}_bytes"] = images[0].stat().st_size if len(images) == 1 else None
    duration_path = build_dir.parent / f"{build_dir.name}.seconds"
    result["build_seconds"] = (
        float(duration_path.read_text(encoding="utf-8").strip())
        if duration_path.exists() else None)
    return result

# Comparison ---------------------------------------------------------------------------------------

def _delta(value, baseline):
    return {
        "absolute" : value - baseline,
        "percent"  : None if baseline == 0 else 100.0*(value - baseline)/baseline,
    }


def make_comparison(root, firmware=None):
    root     = Path(root)
    modes    = {mode: collect_mode(root / mode) for mode in MODES}
    baseline = modes["private"]
    for mode, values in modes.items():
        if values["device"] != baseline["device"]:
            raise ValueError(f"{mode} used {values['device']}, expected {baseline['device']}")
        if values["tool_version"] != baseline["tool_version"]:
            raise ValueError(
                f"{mode} used {values['tool_version']}, expected {baseline['tool_version']}")
    for mode, values in modes.items():
        values["delta_vs_private"] = {
            key: _delta(values[key], baseline[key])
            for key in ("slice_luts", "logic_luts", "lut_memory", "ffs", "ramb36",
                "ramb18", "bram18_equivalent", "dsps", "bit_bytes")
        }
    try:
        git_sha = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
        git_dirty = bool(subprocess.check_output(
            ["git", "status", "--porcelain"], text=True,
            stderr=subprocess.DEVNULL).strip())
    except (OSError, subprocess.CalledProcessError):
        git_sha   = "unknown"
        git_dirty = None
    firmware_sha = None
    if firmware:
        firmware_sha = hashlib.sha256(Path(firmware).read_bytes()).hexdigest()
    return {
        "metadata": {
            "git_sha"            : git_sha,
            "git_dirty"          : git_dirty,
            "firmware_sha256"    : firmware_sha,
            "target"             : "spec_a7_wr_nic",
            "device"             : baseline["device"],
            "tool_version"       : baseline["tool_version"],
            "cache_size_bytes"   : 8192,
            "hyperram_clk_ratio" : "4:1",
            "hyperram_clk_hz"    : 31_250_000,
            "directives"         : {"place": "Explore", "route": "Explore", "post_route_phys_opt": "Explore"},
        },
        "modes": modes,
    }

# Report Output ------------------------------------------------------------------------------------

def write_markdown(comparison, path):
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
        ("WNS (ns)", "wns_ns"),
        ("WHS (ns)", "whs_ns"),
        ("Build time (s)", "build_seconds"),
    )
    lines = [
        "# WR CPU Memory Resource Comparison", "",
        f"Target: `{comparison['metadata']['target']}` / `{comparison['metadata']['device']}`",
        f"Tool: `{comparison['metadata']['tool_version']}`",
        f"Git: `{comparison['metadata']['git_sha']}`"
        f"{' (dirty)' if comparison['metadata']['git_dirty'] else ''}",
        f"Firmware SHA-256: `{comparison['metadata']['firmware_sha256']}`", "",
        f"HyperRAM: `{comparison['metadata']['cache_size_bytes']//1024} KiB cache, "
        f"{comparison['metadata']['hyperram_clk_ratio']}, "
        f"{comparison['metadata']['hyperram_clk_hz']/1e6:g} MHz`", "",
        "| Metric | Private | Integrated | Δ integrated | HyperRAM | Δ HyperRAM |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    modes = comparison["modes"]
    for label, key in metrics:
        private    = modes["private"][key]
        integrated = modes["integrated"][key]
        hyperram   = modes["hyperram"][key]
        if key in modes["integrated"]["delta_vs_private"]:
            idelta = modes["integrated"]["delta_vs_private"][key]
            hdelta = modes["hyperram"]["delta_vs_private"][key]
            idelta_text = f"{idelta['absolute']:+g}" + (
                "" if idelta["percent"] is None else f" ({idelta['percent']:+.1f}%)")
            hdelta_text = f"{hdelta['absolute']:+g}" + (
                "" if hdelta["percent"] is None else f" ({hdelta['percent']:+.1f}%)")
        elif private is not None and integrated is not None and hyperram is not None:
            idelta_text = f"{integrated - private:+.3f}"
            hdelta_text = f"{hyperram - private:+.3f}"
        else:
            idelta_text = "n/a"
            hdelta_text = "n/a"
        lines.append(f"| {label} | {private} | {integrated} | {idelta_text} | {hyperram} | {hdelta_text} |")
    integrated_bram = modes["integrated"]["delta_vs_private"]["bram18_equivalent"]
    hyperram_bram   = modes["hyperram"]["delta_vs_private"]["bram18_equivalent"]
    timing_status = ", ".join(
        f"{mode} {'MET' if modes[mode]['wns_ns'] >= 0 else 'VIOLATED'} "
        f"({modes[mode]['wns_ns']:+.3f} ns WNS)"
        for mode in MODES)
    lines += [
        "", "## Result", "",
        f"Integrated RAM changes block-memory use by "
        f"{integrated_bram['absolute']:+g} BRAM18 equivalents; HyperRAM changes it by "
        f"{hyperram_bram['absolute']:+g}. A negative delta is the FPGA block-memory saving.", "",
        f"Timing status: {timing_status}.", "",
        "All resource figures are from placed designs; timing is from the final timing report.", "",
    ]
    Path(path).write_text("\n".join(lines), encoding="utf-8")


def write_csv(comparison, path):
    keys = (
        "slice_luts", "logic_luts", "lut_memory", "ffs", "ramb36", "ramb18",
        "bram18_equivalent", "dsps", "wns_ns", "tns_ns", "whs_ns", "ths_ns", "bit_bytes",
        "build_seconds",
    )
    with Path(path).open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=("mode",) + keys, lineterminator="\n")
        writer.writeheader()
        for mode in MODES:
            writer.writerow({"mode": mode, **{key: comparison["modes"][mode].get(key) for key in keys}})

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build_root", help="Directory containing private/integrated/hyperram builds.")
    parser.add_argument("--firmware", help="Firmware binary used by all builds.")
    parser.add_argument("--markdown", default="doc/wr_cpu_memory_resource_comparison.md")
    parser.add_argument("--json", default="doc/wr_cpu_memory_resource_comparison.json")
    parser.add_argument("--csv", default="doc/wr_cpu_memory_resource_comparison.csv")
    args       = parser.parse_args()
    comparison = make_comparison(args.build_root, args.firmware)
    Path(args.json).write_text(json.dumps(comparison, indent=2) + "\n", encoding="utf-8")
    write_markdown(comparison, args.markdown)
    write_csv(comparison, args.csv)
    print(f"Wrote {args.markdown}, {args.json}, and {args.csv}.")


if __name__ == "__main__":
    main()
