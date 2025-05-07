#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import shutil
import tarfile
import subprocess

# Toolchain and firmware variables -----------------------------------------------------------------

TOOLCHAIN_URL     = "https://gitlab.com/ohwr/project/wrpc-sw/-/wikis/uploads/9f9224d2249848ed3e854636de9c08dc/riscv-11.2-small.tgz"
TOOLCHAIN_ARCHIVE = "riscv-11.2-small.tgz"
TOOLCHAIN_DIR     = "riscv-11.2-small"

REPO_URL          = "https://gitlab.com/ohwr/project/wrpc-sw.git"
CLONE_DIR         = "wrpc-sw"

COMMIT_HASH       = "5ac04dd53a16f3931c3cb8361dc0d6bdbbf82dc6"
CONFIG_SRC        = "spec_a7_defconfig"

FIRMWARE_SRC      = os.path.join(CLONE_DIR, "wrc.bram")
FIRMWARE_DEST     = "spec_a7_wrc.bram"

SDBFS_DEST        = "sdb-wrpc.bin"
SDBFS_SRC         = "sdbfs"

# Build Helpers/Functions --------------------------------------------------------------------------

def run_command(command, cwd=None):
    """Run a shell command."""
    try:
        subprocess.run(command, cwd=cwd, check=True, shell=True)
    except subprocess.CalledProcessError as e:
        print(f"Error: Command '{e.cmd}' failed with exit code {e.returncode}.")
        exit(1)

def init_riscv_toolchain():
    """Download and extract the RISC-V toolchain, and add it to PATH."""
    if not os.path.exists(TOOLCHAIN_DIR):
        print("Downloading RISC-V toolchain...")
        run_command(f"wget {TOOLCHAIN_URL}")
        print("Extracting toolchain...")
        with tarfile.open(TOOLCHAIN_ARCHIVE, "r:gz") as tar:
            tar.extractall()
    toolchain_bin_path = os.path.join(os.getcwd(), TOOLCHAIN_DIR, "bin")
    os.environ["PATH"] = toolchain_bin_path + os.pathsep + os.environ["PATH"]

def check_riscv_toolchain():
    """Check if riscv32-elf-gcc is available."""
    try:
        subprocess.run(["riscv32-elf-gcc", "--version"], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("Error: riscv32-elf-gcc not found in PATH.")
        exit(1)

def clone_repository():
    """Clone the repository if it does not exist."""
    if not os.path.exists(CLONE_DIR):
        run_command(f"git clone {REPO_URL} --recursive")

def checkout_commit():
    """Checkout the specific commit in the repository."""
    run_command(f"git checkout {COMMIT_HASH}", cwd=CLONE_DIR)

def copy_config_file():
    """Copy the configuration file to the repository."""
    config_dest = os.path.join(CLONE_DIR, "configs/spec_a7_defconfig")
    if not os.path.exists(CONFIG_SRC):
        print(f"Error: Configuration file {CONFIG_SRC} does not exist.")
        exit(1)
    shutil.copy(CONFIG_SRC, config_dest)

def build_firmware():
    """Build the firmware."""
    run_command("make spec_a7_defconfig", cwd=CLONE_DIR)
    run_command("make", cwd=CLONE_DIR)

def copy_firmware():
    """Copy the resulting firmware to the destination."""
    if not os.path.exists(FIRMWARE_SRC):
        print(f"Error: Firmware file {FIRMWARE_SRC} does not exist.")
        exit(1)
    shutil.copy(FIRMWARE_SRC, FIRMWARE_DEST)

def build_sdbfs():
    """Build the SDB filesystem."""
    sdbfs_tool_path = os.path.join(CLONE_DIR, "tools", "gensdbfs")
    sdbfs_src_path = os.path.abspath(SDBFS_SRC)
    sdbfs_dest_path = os.path.abspath(SDBFS_DEST)

    if not os.path.exists(sdbfs_tool_path):
        print(f"Error: SDBFS tool {sdbfs_tool_path} does not exist.")
        exit(1)

    tools_path = os.path.join(CLONE_DIR, "tools")
    os.environ["PATH"] = tools_path + os.pathsep + os.environ["PATH"]

    # Run the command to generate SDB filesystem
    run_command(f"./gensdbfs -b 65536 {sdbfs_src_path} {sdbfs_dest_path}", cwd=tools_path)


# Main ---------------------------------------------------------------------------------------------

def main():
    init_riscv_toolchain()
    check_riscv_toolchain()
    clone_repository()
    checkout_commit()
    copy_config_file()
    build_firmware()
    copy_firmware()
    build_sdbfs()
    print("Build process completed successfully.")

if __name__ == "__main__":
    main()
