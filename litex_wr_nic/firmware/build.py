#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import sys
import shutil
import tarfile
import argparse
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from litex.build import tools

from litex_wr_nic.gateware.wr_cpu import WR_CPU_TYPES, wr_cpu_firmware_filename
from litex_wr_nic.wr_boot import write_boot_image

# Toolchain and firmware variables -----------------------------------------------------------------

TOOLCHAIN_URL     = "https://gitlab.com/ohwr/project/wrpc-sw/-/wikis/uploads/9f9224d2249848ed3e854636de9c08dc/riscv-11.2-small.tgz"
TOOLCHAIN_ARCHIVE = "riscv-11.2-small.tgz"
TOOLCHAIN_DIR     = "riscv-11.2-small"

REPO_URL          = "https://gitlab.com/ohwr/project/wrpc-sw.git"
CLONE_DIR         = "wrpc-sw"

COMMIT_HASH       = "baf7749610b2880bf243b38a9a1608af8e0e688d"
CONFIG_SRC        = "spec_a7_defconfig"

FIRMWARE_SRC       = os.path.join(CLONE_DIR, "wrc.bram")
FIRMWARE_BIN_SRC   = os.path.join(CLONE_DIR, "wrc.bin")

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

def checkout_commit(target="spec_a7"):
    """Checkout the specific commit in the repository."""
    # Ensure no kp/ki modifications.
    run_command(f"git checkout softpll/spll_main.c", cwd=CLONE_DIR)
    run_command(f"git checkout {COMMIT_HASH}", cwd=CLONE_DIR)
    # These files receive the project-local CPU-profile overlay below. Restore
    # only those owned files so repeated uRV/VexRiscv builds cannot leak flags.
    run_command(
        f"git checkout {COMMIT_HASH} -- Makefile arch/risc-v/crt0.S arch/risc-v/irq_helper.c",
        cwd=CLONE_DIR)

    # For Acorn: adapts kp/ki.
    if target != "spec_a7":
        tools.replace_in_file(f"{CLONE_DIR}/softpll/spll_main.c", "s->pi.kp = -1100;", "s->pi.kp = -150;")
        tools.replace_in_file(f"{CLONE_DIR}/softpll/spll_main.c", "s->pi.ki = -30;", "s->pi.ki = -2;")

def copy_config_file():
    """Copy the configuration file to the repository."""
    config_dest = os.path.join(CLONE_DIR, "configs/spec_a7_defconfig")
    if not os.path.exists(CONFIG_SRC):
        print(f"Error: Configuration file {CONFIG_SRC} does not exist.")
        exit(1)
    shutil.copy(CONFIG_SRC, config_dest)

def configure_cpu_profile(cpu_type):
    """Add the small WRPC CPU abstraction needed by LiteX VexRiscv."""
    makefile   = os.path.join(CLONE_DIR, "Makefile")
    crt0       = os.path.join(CLONE_DIR, "arch/risc-v/crt0.S")
    irq_helper = os.path.join(CLONE_DIR, "arch/risc-v/irq_helper.c")
    tools.replace_in_file(
        makefile,
        "asflags-y = $(archflags-y)\n",
        "asflags-y = $(archflags-y)\nasflags-y += $(WR_CPU_ASFLAGS)\n",
    )
    tools.replace_in_file(
        makefile,
        "cflags-y += $(archflags-y)\n",
        "cflags-y += $(archflags-y)\ncflags-y += $(WR_CPU_CFLAGS)\n",
    )
    tools.replace_in_file(
        crt0,
        "_entry:\n\n    la     gp, _gp",
        "_entry:\n\n"
        "#ifdef WR_CPU_VEXRISCV\n"
        "    /* The LiteX VexRiscv mtvec register has no reset value. */\n"
        "    la     t0, _exception_entry\n"
        "    csrw   mtvec, t0\n"
        "#endif\n\n"
        "    la     gp, _gp",
    )
    tools.replace_in_file(
        irq_helper,
        "void enable_irq(void)\n{\n    unsigned long t;\n\n",
        "void enable_irq(void)\n{\n    unsigned long t;\n\n"
        "#ifdef WR_CPU_VEXRISCV\n"
        "    /* LiteX VexRiscv masks its external interrupt array in CSR BC0. */\n"
        "    asm volatile (\"csrw 0xbc0, %0\" : : \"r\"(1));\n"
        "#endif\n\n",
    )

def build_firmware(cpu_type):
    """Build the firmware."""
    configure_cpu_profile(cpu_type)
    run_command("make clean", cwd=CLONE_DIR)
    run_command("make spec_a7_defconfig", cwd=CLONE_DIR)
    cpu_flags = "-DWR_CPU_VEXRISCV" if cpu_type == "vexriscv" else ""
    run_command(
        f"make WR_CPU_CFLAGS={cpu_flags} WR_CPU_ASFLAGS={cpu_flags}",
        cwd=CLONE_DIR)

def copy_firmware(cpu_type):
    """Copy the resulting firmware to the destination."""
    firmware_dest      = wr_cpu_firmware_filename(cpu_type, "bram")
    firmware_bin_dest  = wr_cpu_firmware_filename(cpu_type, "bin")
    firmware_boot_dest = wr_cpu_firmware_filename(cpu_type, "boot")
    if not os.path.exists(FIRMWARE_SRC):
        print(f"Error: Firmware file {FIRMWARE_SRC} does not exist.")
        exit(1)
    shutil.copy(FIRMWARE_SRC, firmware_dest)
    if not os.path.exists(FIRMWARE_BIN_SRC):
        print(f"Error: Firmware file {FIRMWARE_BIN_SRC} does not exist.")
        exit(1)
    shutil.copy(FIRMWARE_BIN_SRC, firmware_bin_dest)
    write_boot_image(firmware_bin_dest, firmware_boot_dest)

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
    # Keep all historical relative paths stable when this script is invoked
    # from the repository root (for example by the resource-matrix helper).
    os.chdir(Path(__file__).resolve().parent)
    parser = argparse.ArgumentParser(description="LiteX-WR-NIC on Acorn Baseboard Mini.")
    parser.add_argument("--target", default="spec_a7", help="Target Board.", choices=["spec_a7", "acorn"])
    parser.add_argument("--wr-cpu-type", default="urv", choices=WR_CPU_TYPES,
        help="WR CPU firmware profile (default: urv).")
    args = parser.parse_args()

    init_riscv_toolchain()
    check_riscv_toolchain()
    clone_repository()
    checkout_commit(args.target)
    copy_config_file()
    build_firmware(args.wr_cpu_type)
    copy_firmware(args.wr_cpu_type)
    build_sdbfs()
    print("Build process completed successfully.")

if __name__ == "__main__":
    main()
