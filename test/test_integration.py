#!/usr/bin/env python3

import pytest

from litex_wr_nic.integration import (
    prepare_wr_environment,
    preflight_wr_cores,
    resolve_wr_paths,
    validate_wr_platform,
)

BASEBOARD_IO = [
    ("sfp", 0, object()),
    ("sfp", 1, object()),
    ("sata", 0, object()),
]


def test_validate_wr_platform_autoselect_sfp():
    result = validate_wr_platform(variant="baseboard", wr_sfp=None, baseboard_io=BASEBOARD_IO)
    assert result["wr_sfp"] == 0
    assert result["available_sfps"] == [0, 1]
    assert result["auto_selected"] is True


def test_validate_wr_platform_rejects_non_baseboard():
    with pytest.raises(ValueError, match="only supported with --variant=baseboard"):
        validate_wr_platform(variant="m2", wr_sfp=0, baseboard_io=BASEBOARD_IO)


def test_resolve_wr_paths_finds_sibling_checkout(tmp_path):
    root = tmp_path / "litex_m2sdr"
    wr_nic = tmp_path / "litex_wr_nic"
    firmware = wr_nic / "firmware"
    firmware.mkdir(parents=True)
    (firmware / "build.py").write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    (firmware / "spec_a7_wrc.bram").write_text("firmware\n", encoding="utf-8")
    root.mkdir()

    resolved_dir, resolved_firmware = resolve_wr_paths(str(root))

    assert resolved_dir == str(wr_nic)
    assert resolved_firmware == str(firmware / "spec_a7_wrc.bram")


def test_resolve_wr_paths_accepts_repo_root(tmp_path):
    repo = tmp_path / "litex_wr_nic"
    package = repo / "litex_wr_nic"
    firmware = package / "firmware"
    firmware.mkdir(parents=True)
    (firmware / "build.py").write_text("#!/usr/bin/env python3\n", encoding="utf-8")

    resolved_dir, resolved_firmware = resolve_wr_paths(
        root_dir=str(tmp_path),
        wr_nic_dir=str(repo),
        wr_firmware=None,
    )

    assert resolved_dir == str(package)
    assert resolved_firmware is None


def test_preflight_wr_cores_rejects_stale_tree(tmp_path):
    root = tmp_path / "litex_m2sdr"
    (root / "wr-cores").mkdir(parents=True)

    with pytest.raises(ValueError, match="Incompatible local 'wr-cores' tree"):
        preflight_wr_cores(str(root), wr_nic_dir=None)


def test_prepare_wr_environment_status_only_prints(capsys, tmp_path):
    wr_env = prepare_wr_environment(
        root_dir=str(tmp_path),
        variant="baseboard",
        baseboard_io=BASEBOARD_IO,
        with_white_rabbit=False,
        wr_sfp=None,
        wr_nic_dir=None,
        wr_firmware=None,
        wr_firmware_target="acorn",
        build=False,
        status=True,
    )

    captured = capsys.readouterr()
    assert "White Rabbit status:" in captured.out
    assert wr_env["wr_sfp"] is None
