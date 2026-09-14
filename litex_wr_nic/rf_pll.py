#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import math
from pathlib import Path
import re
import time


# LMX2572 Configuration ----------------------------------------------------------------------------

class LMX2572Config:
    """TICS Pro register export for normal, fixed-frequency operation."""
    def __init__(self, registers, reference_frequency=None):
        self.registers = dict(registers)
        self.reference_frequency = reference_frequency

    @classmethod
    def from_file(cls, filename):
        path = Path(filename)
        registers = {}
        section = ""
        reference = None
        for number, line in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
            line = line.strip()
            if not line or line.startswith(("#", ";")):
                continue
            try:
                if path.suffix.lower() == ".tcs":
                    if line.startswith("[") and line.endswith("]"):
                        section = line[1:-1].upper()
                        continue
                    key, value = (part.strip() for part in line.split("=", 1))
                    if section == "SETUP" and key.upper() == "PART" and value != "LMX2572":
                        raise ValueError("expected PART=LMX2572")
                    if section == "FLEX" and key.upper() == "FOSC_FREQ":
                        reference = float(value) * 1e6
                    if section != "MODES" or not key.upper().startswith("VALUE"):
                        continue
                    if not re.fullmatch(r"VALUE\d+", key, re.IGNORECASE):
                        raise ValueError("invalid register entry")
                    word = int(value, 16 if value.lower().startswith("0x") else 10)
                else:
                    label, value = line.split()
                    word = int(value, 16)
                    if label != f"R{word >> 16}":
                        raise ValueError("register label and encoded address differ")
                if not 0 <= word <= 0x7dffff:
                    raise ValueError("expected a write to R0..R125")
                address, value = word >> 16, word & 0xffff
                if address in registers:
                    raise ValueError(f"duplicate R{address}")
                registers[address] = value
            except ValueError as error:
                raise ValueError(f"{path}:{number}: {error}") from error
        if not registers:
            raise ValueError(f"{path}: no registers found")
        return cls(registers, reference)

    def frequencies(self, reference_frequency=25e6):
        """Validate a fixed-frequency image and return nominal PFD/VCO/A/B frequencies.

        These are calculated settings, not measured frequencies or lock status.
        """
        if not math.isfinite(reference_frequency) or not 5e6 <= reference_frequency <= 250e6:
            raise ValueError("reference frequency must be between 5 and 250 MHz")
        if self.reference_frequency is not None and not math.isclose(
                self.reference_frequency, reference_frequency, rel_tol=1e-6):
            raise ValueError(f"configuration expects a {self.reference_frequency/1e6:g} MHz reference; "
                             f"the board supplies {reference_frequency/1e6:g} MHz")
        r = self.registers
        if any(type(a) is not int or not 0 <= a <= 125 or type(v) is not int or not 0 <= v <= 0xffff
               for a, v in r.items()):
            raise ValueError("configuration requires R0..R125 addresses and 16-bit values")
        required = (0, 9, 10, 11, 12, 34, 36, 38, 39, 42, 43, 44, 45, 46, 75, 114)
        missing = [address for address in required if address not in r]
        if missing:
            raise ValueError(f"configuration is missing registers: {missing}")
        if r[0] & ((1 << 15) | (1 << 14) | (1 << 11) | 3) or r[114] & (1 << 10):
            raise ValueError("configuration must use normal operation, without ramp, FSK, phase sync, "
                             "address hold, reset or powerdown")
        if not r[0] & (1 << 3):
            raise ValueError("R0 must enable VCO calibration (FCAL_EN)")
        pre, post, mult = r[12] & 0xfff, (r[11] >> 4) & 0xff, (r[10] >> 7) & 0x1f
        if not pre or not post or mult not in (1, 3, 4, 5, 6, 7):
            raise ValueError("invalid reference divider or multiplier")
        doubler = 2 if r[9] & (1 << 12) else 1
        if doubler == 2 and reference_frequency > 125e6:
            raise ValueError("reference doubler requires at most 125 MHz")
        mult_input = reference_frequency * doubler / pre
        if mult != 1 and not (10e6 <= mult_input <= 40e6 and 60e6 <= mult_input*mult <= 150e6):
            raise ValueError("reference multiplier frequency is out of range")
        pfd = mult_input * mult / post
        order = r[44] & 7
        pfd_min = 0.25e6 if order == 0 else 5e6
        pfd_max = (250e6, 200e6, 200e6, 160e6, 120e6)
        if order > 4 or not pfd_min <= pfd <= pfd_max[order]:
            raise ValueError("PFD frequency or MASH order is out of range")
        numerator, denominator = (r[42] << 16) | r[43], (r[38] << 16) | r[39]
        if not denominator or numerator >= denominator or (not order and numerator):
            raise ValueError("invalid fractional divider")
        n = ((r[34] & 7) << 16) | r[36]
        vco = pfd * (n + (numerator / denominator if order else 0))
        if not 3.2e9 <= vco <= 6.4e9:
            raise ValueError(f"requested VCO frequency {vco/1e6:g} MHz is outside 3200..6400 MHz; "
                             "check the reference frequency and PLL dividers")
        dividers = {0: 2, 1: 4, 3: 8, 5: 16, 7: 32, 9: 64, 12: 128, 14: 256}
        divider = dividers.get((r[75] >> 6) & 0x1f)
        outputs = []
        for mux, powered_down in (((r[45] >> 11) & 3, r[44] & (1 << 6)),
                                  (r[46] & 3, r[44] & (1 << 7))):
            if powered_down or mux == 3:
                outputs.append(None)
            elif mux == 1:
                outputs.append(vco)
            elif mux == 0 and divider is not None:
                outputs.append(vco / divider)
            else:
                raise ValueError("invalid channel divider or non-RF output selected")
        return dict(reference=reference_frequency, pfd=pfd, vco=vco, out_a=outputs[0], out_b=outputs[1])

    def writes(self):
        # TI SNAS740B 7.5.1: reset, then descending addresses, with R0/FCAL last.
        # R107..R113 are read-only and occur in TICS Pro exports too.
        yield 0, 0x221e
        for address in sorted(self.registers, reverse=True):
            if address not in range(107, 114):
                yield address, self.registers[address]


# LMX2572 Host Control -----------------------------------------------------------------------------

class LMX2572:
    def __init__(self, bus, name="rf_out_pll", timeout=1.0):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be positive and finite")
        self.bus     = bus
        self.name    = name
        self.timeout = timeout

    def reg(self, suffix):
        return getattr(self.bus.regs, f"{self.name}_{suffix}")

    def wait_done(self, check_error=True):
        deadline = time.monotonic() + self.timeout
        while True:
            status = self.reg("status").read()
            if status & 1:
                if check_error and status & 4:
                    raise RuntimeError("RF PLL SPI controller rejected the transfer")
                return
            if time.monotonic() >= deadline:
                raise TimeoutError("RF PLL SPI transfer timed out")
            time.sleep(0.001)

    def write_reg(self, addr, value):
        if type(addr) is not int or not 0 <= addr <= 125:
            raise ValueError("register address must be in R0..R125")
        if type(value) is not int or not 0 <= value <= 0xffff:
            raise ValueError("register value must fit in 16 bits")
        if addr in range(107, 114):
            raise ValueError(f"R{addr} is read-only")
        self.wait_done(check_error=False)
        # Recover from a previous user's manual CS or loopback setting.
        self.reg("cs").write(1)
        self.reg("loopback").write(0)
        self.reg("mosi").write((addr << 16) | value)
        self.reg("control").write((24 << 8) | 1)
        self.wait_done()

    def load(self, config, reference_frequency=25e6):
        if not isinstance(config, LMX2572Config):
            config = LMX2572Config.from_file(config)
        frequencies = config.frequencies(reference_frequency)
        if hasattr(self.bus.regs, f"{self.name}_sync"):
            self.reg("sync").write(0)
        # CE is tied high on SPEC-A7; allow TI's 500 us LDO startup interval.
        time.sleep(0.0005)
        for address, value in config.writes():
            self.write_reg(address, value)
        return frequencies

    def toggle_sync(self):
        if not hasattr(self.bus.regs, f"{self.name}_sync"):
            raise RuntimeError("this bitstream does not expose RF PLL SYNC control")
        self.reg("sync").write(0)
        try:
            self.reg("sync").write(1)
            time.sleep(0.000001)
        finally:
            self.reg("sync").write(0)
