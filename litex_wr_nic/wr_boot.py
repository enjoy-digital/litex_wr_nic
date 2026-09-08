#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""White Rabbit CPU flash boot-image format and packaging helpers."""

import zlib
import struct
import argparse
from pathlib import Path

# Constants ----------------------------------------------------------------------------------------

WR_BOOT_MAGIC        = b"WRCB"
WR_BOOT_VERSION      = 1
WR_BOOT_HEADER       = struct.Struct("<4sIII")
WR_BOOT_MAX_PAYLOAD  = 128 * 1024
WR_BOOT_FLASH_OFFSET = 0x002f_0000
WR_SDB_FLASH_OFFSET  = 0x002e_0000
WR_SDB_MAX_SIZE      = 64 * 1024
SPEC_A7_FLASH_SIZE   = 16 * 1024 * 1024

# Boot Image Helpers -------------------------------------------------------------------------------

def build_boot_image(payload, max_payload=WR_BOOT_MAX_PAYLOAD, pad_to_max=False):
    payload = bytes(payload)
    if not payload:
        raise ValueError("WR CPU boot payload must not be empty")
    padded = payload + bytes((-len(payload)) % 4)
    if len(padded) > max_payload:
        raise ValueError(
            f"WR CPU boot payload is {len(padded)} bytes; maximum is {max_payload}")
    if pad_to_max:
        padded += bytes(max_payload - len(padded))
    crc = zlib.crc32(padded) & 0xffff_ffff
    return WR_BOOT_HEADER.pack(WR_BOOT_MAGIC, WR_BOOT_VERSION, len(padded), crc) + padded


def parse_boot_image(image, max_payload=WR_BOOT_MAX_PAYLOAD):
    image = bytes(image)
    if len(image) < WR_BOOT_HEADER.size:
        raise ValueError("WR CPU boot image is shorter than its header")
    magic, version, length, expected_crc = WR_BOOT_HEADER.unpack_from(image)
    if magic != WR_BOOT_MAGIC:
        raise ValueError("invalid WR CPU boot-image magic")
    if version != WR_BOOT_VERSION:
        raise ValueError(f"unsupported WR CPU boot-image version {version}")
    if length == 0 or length % 4:
        raise ValueError("WR CPU boot-image payload length must be non-zero and 4-byte aligned")
    if length > max_payload:
        raise ValueError("WR CPU boot-image payload exceeds the CPU memory window")
    if len(image) != WR_BOOT_HEADER.size + length:
        raise ValueError("WR CPU boot-image length does not match its header")
    payload    = image[WR_BOOT_HEADER.size:]
    actual_crc = zlib.crc32(payload) & 0xffff_ffff
    if actual_crc != expected_crc:
        raise ValueError(
            f"WR CPU boot-image CRC mismatch: expected 0x{expected_crc:08x}, got 0x{actual_crc:08x}")
    return payload


def write_boot_image(input_path, output_path):
    payload = Path(input_path).read_bytes()
    # Match the deterministic zero-filled tail of the private and integrated
    # RAM images. This also initializes the complete HyperRAM CPU window before
    # reset is released.
    image = build_boot_image(payload, pad_to_max=True)
    Path(output_path).write_bytes(image)
    return len(image)


def validate_flash_layout(bitstream_path, sdb_path, boot_path=None, flash_size=SPEC_A7_FLASH_SIZE):
    """Validate the fixed SPEC-A7 bitstream/SDB/WR-boot flash layout."""
    regions = [
        ("FPGA bitstream", 0, Path(bitstream_path).stat().st_size),
        ("WR SDB", WR_SDB_FLASH_OFFSET, Path(sdb_path).stat().st_size),
    ]
    if regions[1][2] > WR_SDB_MAX_SIZE:
        raise ValueError(f"WR SDB image exceeds its {WR_SDB_MAX_SIZE}-byte slot")
    if boot_path is not None:
        boot_size = Path(boot_path).stat().st_size
        parse_boot_image(Path(boot_path).read_bytes())
        regions.append(("WR CPU boot image", WR_BOOT_FLASH_OFFSET, boot_size))
    for name, offset, size in regions:
        if offset + size > flash_size:
            raise ValueError(f"{name} exceeds the {flash_size}-byte SPI flash")
    ordered = sorted(regions, key=lambda region: region[1])
    for first, second in zip(ordered, ordered[1:]):
        if first[1] + first[2] > second[1]:
            raise ValueError(f"{first[0]} overlaps {second[0]} in SPI flash")
    return regions

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Package a WR CPU binary for FPGA flash boot.")
    parser.add_argument("input", help="Raw WR CPU firmware binary.")
    parser.add_argument("output", help="Packaged boot image.")
    args = parser.parse_args()
    size = write_boot_image(args.input, args.output)
    print(f"Wrote {size} bytes to {args.output}.")


if __name__ == "__main__":
    main()
