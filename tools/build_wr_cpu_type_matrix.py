#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Build the reproducible SPEC-A7 uRV/VexRiscv WR CPU matrix."""

import sys
import time
import argparse
import subprocess
from pathlib import Path

# Constants ----------------------------------------------------------------------------------------

CONFIGS = {
    "urv-integrated"           :             ("urv",       None,   "integrated"),
    "vexriscv-lite-integrated" :   ("vexriscv", "lite", "integrated"),
    "urv-hyperram"             :               ("urv",       None,   "hyperram"),
    "vexriscv-lite-hyperram"   :     ("vexriscv", "lite", "hyperram"),
}
REPO_ROOT = Path(__file__).resolve().parents[1]

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", default="build/wr_cpu_types")
    parser.add_argument("--skip-firmware-build", action="store_true")
    parser.add_argument("--configs", nargs="+", choices=CONFIGS, default=tuple(CONFIGS),
        help="Build only selected configurations; comparison still requires all reports.")
    args = parser.parse_args()

    if not args.skip_firmware_build:
        for cpu_type in ("urv", "vexriscv"):
            subprocess.run([
                sys.executable, str(REPO_ROOT / "litex_wr_nic/firmware/build.py"),
                "--target", "spec_a7", "--wr-cpu-type", cpu_type,
            ], check=True, cwd=REPO_ROOT)

    root = Path(args.output_root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    for name in args.configs:
        cpu_type, variant, memory = CONFIGS[name]
        output_dir                = root / name
        log_path                  = root / f"{name}.log"
        command = [
            sys.executable, str(REPO_ROOT / "spec_a7_wr_nic.py"),
            "--build", "--skip-firmware-build", "--skip-software-headers",
            "--wr-cpu-type", cpu_type,
            "--wr-cpu-memory", memory,
            "--output-dir", str(output_dir),
        ]
        if variant is not None:
            command += ["--wr-cpu-variant", variant]
        print(f"Building {name} -> {output_dir}", flush=True)
        started = time.monotonic()
        with log_path.open("w", encoding="utf-8") as log:
            subprocess.run(command,
                check  = True,
                cwd    = REPO_ROOT,
                stdout = log,
                stderr = subprocess.STDOUT,
            )
        duration = time.monotonic() - started
        (root / f"{name}.seconds").write_text(f"{duration:.3f}\n", encoding="utf-8")
        print(f"Completed {name} in {duration:.1f}s (log: {log_path})", flush=True)

    subprocess.run([
        sys.executable, str(REPO_ROOT / "tools/compare_wr_cpu_types.py"), str(root),
        "--firmware-dir", str(REPO_ROOT / "litex_wr_nic/firmware"),
    ], check=True, cwd=REPO_ROOT)


if __name__ == "__main__":
    main()
