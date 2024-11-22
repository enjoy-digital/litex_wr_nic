#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import sys
import argparse
import subprocess

# Helper Functions ---------------------------------------------------------------------------------

def ensure_sudo():
    """Ensure the script is running with root permissions."""
    if os.geteuid() != 0:
        print("This script requires root permissions. Rerunning with sudo...")
        try:
            subprocess.check_call(["sudo", sys.executable] + sys.argv)
        except subprocess.CalledProcessError as e:
            print(f"Error: Failed to execute with sudo. {e}")
        sys.exit(0)

def enable_pps(ptp_path):
    print(f"Enabling PPS output on SDP0 for {ptp_path}...")
    try:
        with open(f"{ptp_path}/pps_enable", "w") as f:
            f.write("1")
        with open(f"{ptp_path}/pins/SDP0", "w") as f:
            f.write("2 0")
        with open(f"{ptp_path}/period", "w") as f:
            f.write("0 0 0 1 0")
        print("PPS output enabled successfully.")
    except IOError as e:
        print(f"Error: {e}")
        print("Make sure the PTP device exists and is accessible.")

def disable_pps(ptp_path):
    print(f"Disabling PPS output on SDP0 for {ptp_path}...")
    try:
        with open(f"{ptp_path}/pps_enable", "w") as f:
            f.write("0")
        with open(f"{ptp_path}/pins/SDP0", "w") as f:
            f.write("0 0")
        with open(f"{ptp_path}/period", "w") as f:
            f.write("0 0 0 0 0")
        print("PPS output disabled successfully.")
    except IOError as e:
        print(f"Error: {e}")
        print("Make sure the PTP device exists and is accessible.")

# Main ---------------------------------------------------------------------------------------------

def main():
    ensure_sudo()

    parser = argparse.ArgumentParser(description="Control PPS output on SDP0 for Intel I225.")
    parser.add_argument("--ptp",     type=str, default="ptp0", help="Specify the PTP device (default: ptp0).")
    parser.add_argument("--enable",  action="store_true",      help="Enable PPS output.")
    parser.add_argument("--disable", action="store_true",      help="Disable PPS output.")

    args = parser.parse_args()

    ptp_path = f"/sys/class/ptp/{args.ptp}"

    if not os.path.exists(ptp_path):
        print(f"Error: PTP device {args.ptp} does not exist.")
        return

    if args.enable and args.disable:
        print("Error: Cannot enable and disable PPS simultaneously.")
        return

    if args.enable:
        enable_pps(ptp_path)
    elif args.disable:
        disable_pps(ptp_path)
    else:
        print("Error: Specify --enable or --disable.")

if __name__ == "__main__":
    main()
