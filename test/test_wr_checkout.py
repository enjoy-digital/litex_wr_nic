"""A build must never silently reuse a different WR-core or submodule revision."""
import subprocess

import pytest

from litex_wr_nic.gateware import wr_common


def git(path, *args):
    return subprocess.check_output([
        "git", "-C", str(path), "-c", "user.name=WR test",
        "-c", "user.email=wr-test@example.invalid",
        "-c", "protocol.file.allow=always", *args], text=True).strip()


def commit(path, text):
    (path / "revision").write_text(text)
    git(path, "add", ".")
    git(path, "commit", "-qm", text)
    return git(path, "rev-parse", "HEAD")


def test_reused_core_rejects_wrong_revision_without_discarding_edits(tmp_path, monkeypatch):
    core = tmp_path / "wr-cores"
    core.mkdir()
    git(core, "init", "-q")
    old = commit(core, "old")
    current = commit(core, "current")
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(wr_common, "WR_CORES_SHA1", current)
    wr_common.wr_core_init()
    git(core, "checkout", "-q", old)
    (core / "revision").write_text("local edit")
    with pytest.raises(RuntimeError, match="expected " + current):
        wr_common.wr_core_init()
    assert git(core, "rev-parse", "HEAD") == old
    assert (core / "revision").read_text() == "local edit"


def test_reused_core_rejects_mismatched_submodule(tmp_path, monkeypatch):
    dependency = tmp_path / "dependency"
    dependency.mkdir()
    git(dependency, "init", "-q")
    old = commit(dependency, "old")
    commit(dependency, "current")
    core = tmp_path / "wr-cores"
    core.mkdir()
    git(core, "init", "-q")
    git(core, "submodule", "add", "-q", str(dependency), "dependency")
    current = commit(core, "core")
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(wr_common, "WR_CORES_SHA1", current)
    wr_common.wr_core_init()
    git(core / "dependency", "checkout", "-q", old)
    with pytest.raises(RuntimeError, match="submodules do not match"):
        wr_common.wr_core_init()
    assert git(core / "dependency", "rev-parse", "HEAD") == old
