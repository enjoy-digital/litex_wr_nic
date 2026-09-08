#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import struct

import pytest

from litex_wr_nic.wr_boot import (
    WR_BOOT_HEADER,
    WR_BOOT_MAX_PAYLOAD,
    build_boot_image,
    parse_boot_image,
    validate_flash_layout,
    write_boot_image,
)

# Boot Image Tests ---------------------------------------------------------------------------------

def test_boot_image_round_trip_and_padding():
    image   = build_boot_image(b"\x13\x00\x00\x00\xaa")
    payload = parse_boot_image(image)
    assert len(image) == WR_BOOT_HEADER.size + 8
    assert payload == b"\x13\x00\x00\x00\xaa\x00\x00\x00"


def test_written_boot_image_initializes_complete_cpu_window(tmp_path):
    raw  = tmp_path / "wrc.bin"
    boot = tmp_path / "wrc.boot"
    raw.write_bytes(b"\x13\x00\x00\x00")
    write_boot_image(raw, boot)
    payload = parse_boot_image(boot.read_bytes())
    assert len(payload) == WR_BOOT_MAX_PAYLOAD
    assert payload[:4] == raw.read_bytes()
    assert not any(payload[4:])


@pytest.mark.parametrize("payload", [b"", bytes(WR_BOOT_MAX_PAYLOAD + 1)])
def test_boot_image_rejects_invalid_payload_size(payload):
    with pytest.raises(ValueError):
        build_boot_image(payload)


def test_boot_image_rejects_corruption():
    image = bytearray(build_boot_image(b"firmware"))
    image[-1] ^= 0x80
    with pytest.raises(ValueError, match="CRC mismatch"):
        parse_boot_image(image)


def test_boot_image_rejects_unaligned_header_length():
    image = bytearray(build_boot_image(b"firmware"))
    struct.pack_into("<I", image, 8, 3)
    with pytest.raises(ValueError, match="4-byte aligned"):
        parse_boot_image(image)


def test_flash_layout_rejects_bitstream_overlap(tmp_path):
    bitstream = tmp_path / "gateware.bin"
    sdb       = tmp_path / "sdb.bin"
    bitstream.write_bytes(bytes(0x002e_0001))
    sdb.write_bytes(bytes(64 * 1024))
    with pytest.raises(ValueError, match="overlaps"):
        validate_flash_layout(bitstream, sdb)


def test_profile_metadata_and_validation():
    from litex_wr_nic.wr_boot import inspect_boot_image
    for cpu in ("urv", "vexriscv"):
        image = build_boot_image(b"test", cpu_type=cpu)
        assert len(image) == 36
        assert inspect_boot_image(image, cpu_type=cpu) == dict(version=2, cpu_type=cpu, payload=b"test")
        other = "urv" if cpu == "vexriscv" else "vexriscv"
        with pytest.raises(ValueError, match="profile mismatch"):
            parse_boot_image(image, cpu_type=other)
    for offset, match in ((16, "profile"), (20, "ABI"), (24, "address"), (28, "address")):
        damaged = bytearray(image)
        damaged[offset] ^= 0x80
        with pytest.raises(ValueError, match=match):
            parse_boot_image(damaged)
