#!/usr/bin/env python3

import os
import subprocess
import argparse

def init_wr_cores():
    print("Cloning wr-cores repository...")
    subprocess.run(["git", "clone", "https://ohwr.org/project/wr-cores.git"])
    os.chdir("wr-cores")
    print("Checking out the specified commit...")
    subprocess.run(["git", "checkout", "39825ec55291cb12492090093f27a50f9d0b73d9", "-b", "wrpc-v5"])
    print("Updating submodules...")
    subprocess.run(["git", "submodule", "update", "--init"])
    print("wr-cores initialization complete.")

def main():
    parser = argparse.ArgumentParser(description="Initialize wr-cores or RISC-V toolchain")
    parser.add_argument("--wr-cores", action="store_true", help="Initialize the wr-cores repository")

    args = parser.parse_args()

    if args.wr_cores:
        init_wr_cores()

    if args.riscv:
        init_riscv_toolchain()

if __name__ == "__main__":
    main()
