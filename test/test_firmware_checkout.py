#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import importlib.util
from pathlib import Path
import subprocess


def git(path, *args):
    return subprocess.check_output(["git", "-C", str(path),
        "-c", "user.name=WR test", "-c", "user.email=wr-test@example.invalid",
        "-c", "protocol.file.allow=always", *args], text=True).strip()


def commit(path, message):
    git(path, "add", ".")
    git(path, "commit", "-qm", message)
    return git(path, "rev-parse", "HEAD")


def test_reused_firmware_checkout_restores_pinned_submodule_and_gains(tmp_path, monkeypatch):
    dependency = tmp_path / "ppsi-source"
    dependency.mkdir()
    git(dependency, "init", "-q")
    (dependency / "version").write_text("pinned\n")
    pinned = commit(dependency, "Pinned PPSI")
    (dependency / "version").write_text("different\n")
    newer = commit(dependency, "Different PPSI")

    checkout = tmp_path / "wrpc-sw"
    checkout.mkdir()
    git(checkout, "init", "-q")
    for name in ("Makefile", "arch/risc-v/crt0.S", "arch/risc-v/irq_helper.c",
                 "include/board.h", "dev/sfp.c"):
        path = checkout / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("fixture\n")
    gains = checkout / "softpll/spll_main.c"
    gains.parent.mkdir()
    original = "#define MPLL_FREQ_PRELOCK_GAIN_BOOST 20\ns->pi.kp = -1100;\ns->pi.ki = -30;\n"
    gains.write_text(original)
    git(checkout, "submodule", "add", "-q", str(dependency), "ppsi")
    git(checkout / "ppsi", "checkout", "-q", pinned)
    parent = commit(checkout, "Pin firmware dependencies")
    git(checkout / "ppsi", "checkout", "-q", newer)

    path = Path(__file__).resolve().parents[1] / "litex_wr_nic/firmware/build.py"
    spec = importlib.util.spec_from_file_location("wr_firmware_build", path)
    build = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(build)
    monkeypatch.setattr(build, "CLONE_DIR", str(checkout))
    monkeypatch.setattr(build, "COMMIT_HASH", parent)

    build.checkout_commit("acorn")
    assert git(checkout / "ppsi", "rev-parse", "HEAD") == pinned
    assert "s->pi.kp = -150;" in gains.read_text()
    assert "s->pi.ki = -2;" in gains.read_text()
    assert "#define MPLL_FREQ_PRELOCK_GAIN_BOOST 20" in gains.read_text()
    build.checkout_commit("spec_a7")
    assert gains.read_text() == original
