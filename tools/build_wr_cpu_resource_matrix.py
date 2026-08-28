#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Build the three reproducible SPEC-A7 WR CPU memory configurations."""

import sys
import time
import argparse
import subprocess
from pathlib import Path

# Constants ----------------------------------------------------------------------------------------

MODES     = ("private", "integrated", "hyperram")
REPO_ROOT = Path(__file__).resolve().parents[1]

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", default="build/wr_cpu_resources")
    parser.add_argument("--skip-firmware-build", action="store_true")
    parser.add_argument("--modes", nargs="+", choices=MODES, default=MODES,
        help="Build only selected modes (completed reports for all modes are still compared).")
    args = parser.parse_args()

    if not args.skip_firmware_build:
        subprocess.run([
            sys.executable, str(REPO_ROOT / "litex_wr_nic/firmware/build.py"),
            "--target", "spec_a7",
        ], check=True, cwd=REPO_ROOT)

    root = Path(args.output_root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    for mode in args.modes:
        output_dir = root / mode
        log_path   = root / f"{mode}.log"
        command = [
            sys.executable, str(REPO_ROOT / "spec_a7_wr_nic.py"),
            "--build", "--skip-firmware-build",
            "--skip-software-headers",
            "--wr-cpu-memory", mode, "--output-dir", str(output_dir),
        ]
        print(f"Building {mode} WR CPU memory -> {output_dir}", flush=True)
        started = time.monotonic()
        with log_path.open("w", encoding="utf-8") as log:
            subprocess.run(command,
                check  = True,
                cwd    = REPO_ROOT,
                stdout = log,
                stderr = subprocess.STDOUT,
            )
        duration = time.monotonic() - started
        (root / f"{mode}.seconds").write_text(f"{duration:.3f}\n", encoding="utf-8")
        print(f"Completed {mode} in {duration:.1f}s (log: {log_path})", flush=True)

    subprocess.run([
        sys.executable, str(REPO_ROOT / "tools/compare_wr_cpu_resources.py"), str(root),
        "--firmware", str(REPO_ROOT / "litex_wr_nic/firmware/spec_a7_wrc.bin"),
    ], check=True, cwd=REPO_ROOT)


if __name__ == "__main__":
    main()
