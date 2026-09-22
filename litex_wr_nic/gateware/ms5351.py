#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.cores.i2c import I2CMasterMachine

# MS5351 PLL Tuner ---------------------------------------------------------------------------------

# The MS5351M (Si5351A compatible) clock generator multiplies its crystal with a
# fractional-N PLL: f_vco = f_xtal * (P1 + 512 + P2/P3) / 128. Writing the PLL
# feedback registers changes the VCO frequency without a PLL reset, so a fine
# fractional update acts as a frequency actuator for every output of that PLL.
#
# The tuner keeps the integer multiplier configured by the board (P1) and
# spans one integer step with a 21-bit code: bit 20 selects P1 or P1 - 1 and
# the low 20 bits are P2 with P3 = 2**20 - 1. A code of 2**20 is the nominal
# frequency; the range is +/- 1/(128 * multiplier), i.e. +/- 217 ppm for a
# 900 MHz VCO from 25 MHz, with one code equal to about 0.2 ppb.

MS5351_PLLA_REGISTER = 26
MS5351_PLLB_REGISTER = 34
MS5351_CODE_WIDTH    = 21
MS5351_CODE_NOMINAL  = 1 << (MS5351_CODE_WIDTH - 1)


def ms5351_pll_registers(multiplier, code):
    """Return the eight PLL register bytes selecting ``code`` around ``multiplier``."""
    if not 0 <= code < (1 << MS5351_CODE_WIDTH):
        raise ValueError("MS5351 tuning code is 21 bits wide.")
    if not 15 <= multiplier <= 90:
        raise ValueError("MS5351 PLL multipliers range from 15 to 90.")
    p1 = 128*multiplier - 512 - 1 + (code >> 20)
    p2 = code & 0xfffff
    p3 = (1 << 20) - 1
    return bytes([
        (p3 >> 8) & 0xff, p3 & 0xff,
        (p1 >> 16) & 0x03, (p1 >> 8) & 0xff, p1 & 0xff,
        ((p3 >> 12) & 0xf0) | ((p2 >> 16) & 0x0f), (p2 >> 8) & 0xff, p2 & 0xff,
    ])


class MS5351PLLTuner(LiteXModule):
    """Apply a 21-bit tuning code to one MS5351 PLL over I2C.

    ``code`` is sampled whenever the tuner is idle and differs from the last
    applied value; a newer code supersedes a pending one. The first update and
    every 256th one rewrite all eight registers, the others start at the first
    register that changed (P1 when bit 20 toggles, else the P2 bytes) so the
    servo update time stays below 150 us at 400 kHz. ``valid`` reports that the
    device holds the fractional P3 denominator; clear ``enable`` to release the
    bus for another master.
    """
    def __init__(self, sys_clk_freq, i2c_freq=400e3, i2c_address=0x60, multiplier=36, pll="A",
        select_cycles=64, release_cycles=256):
        if pll not in ("A", "B"):
            raise ValueError("MS5351 PLL must be A or B.")
        self.enable  = Signal()
        self.code    = Signal(MS5351_CODE_WIDTH, reset=MS5351_CODE_NOMINAL)
        self.busy    = Signal()
        self.valid   = Signal()
        self.current = Signal(MS5351_CODE_WIDTH, reset=MS5351_CODE_NOMINAL)
        self.updates = Signal(32)
        self.errors  = Signal(32)
        self.scl_o   = Signal(reset=1)
        self.sda_o   = Signal(reset=1)
        self.sda_i   = Signal()

        # # #

        base_register = {"A": MS5351_PLLA_REGISTER, "B": MS5351_PLLB_REGISTER}[pll]
        base_p1       = 128*multiplier - 512 - 1

        self.i2c = i2c = I2CMasterMachine(clock_width=16)
        self.comb += [
            i2c.cg.load.eq(int(sys_clk_freq/(2*i2c_freq)) - 1),
            self.scl_o.eq(i2c.scl_o),
            self.sda_o.eq(i2c.sda_o),
            i2c.sda_i.eq(self.sda_i),
        ]

        # Register image of the code being written.
        code   = Signal(MS5351_CODE_WIDTH)
        p1     = Signal(18)
        p2     = Signal(20)
        first  = Signal(4) # First data byte written, 0-7.
        index  = Signal(4) # Byte index in the transaction, 0-9.
        failed = Signal()
        self.comb += [
            p1.eq(base_p1 + code[20]),
            p2.eq(code[:20]),
        ]
        data = Array([
            Constant(i2c_address << 1, 8),
            base_register + first,
            Constant(0xff, 8),                # P3[15:8]
            Constant(0xff, 8),                # P3[7:0]
            p1[16:18],                        # P1[17:16]
            p1[8:16],                         # P1[15:8]
            p1[0:8],                          # P1[7:0]
            Cat(p2[16:20], Constant(0xf, 4)), # P3[19:16] | P2[19:16]
            p2[8:16],                         # P2[15:8]
            p2[0:8],                          # P2[7:0]
        ])

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.comb += self.busy.eq(~fsm.ongoing("IDLE"))
        fsm.act("IDLE",
            If(~self.enable,
                NextValue(self.valid, 0),
            ).Elif(~self.valid | (self.code != self.current),
                NextValue(code, self.code),
                NextValue(index, 0),
                NextValue(failed, 0),
                # Full rewrite: first update, periodic refresh or P1 change.
                If(~self.valid | (self.updates[0:8] == 0),
                    NextValue(first, 0),
                ).Elif(self.code[20] != self.current[20],
                    NextValue(first, 2),
                ).Else(
                    NextValue(first, 5),
                ),
                NextState("SELECT"),
            ),
        )
        # Let an upstream analog bus multiplexer settle before START.
        fsm.delayed_enter("SELECT", "START", select_cycles)
        fsm.act("START",
            i2c.start.eq(1),
            NextState("START-WAIT"),
        )
        fsm.act("START-WAIT",
            If(i2c.idle,
                NextState("BYTE"),
            ),
        )
        fsm.act("BYTE",
            NextValue(i2c.data, data[index]),
            NextState("WRITE"),
        )
        fsm.act("WRITE",
            i2c.write.eq(1),
            NextState("WRITE-WAIT"),
        )
        fsm.act("WRITE-WAIT",
            If(i2c.idle,
                If(~i2c.ack,
                    NextValue(failed, 1),
                    NextState("STOP"),
                ).Elif(index == 9,
                    NextState("STOP"),
                ).Else(
                    # After the register address, skip to the first changed byte.
                    NextValue(index, Mux(index == 1, 2 + first, index + 1)),
                    NextState("BYTE"),
                ),
            ),
        )
        fsm.act("STOP",
            i2c.stop.eq(1),
            NextState("STOP-WAIT"),
        )
        fsm.act("STOP-WAIT",
            If(i2c.idle,
                If(failed,
                    NextValue(self.errors, self.errors + 1),
                    NextValue(self.valid, 0),
                ).Else(
                    NextValue(self.current, code),
                    NextValue(self.updates, self.updates + 1),
                    NextValue(self.valid, 1),
                ),
                NextState("RELEASE"),
            ),
        )
        # Keep the bus through its free time, so a shared multiplexer is not
        # switched while the device still samples STOP.
        fsm.delayed_enter("RELEASE", "IDLE", release_cycles)
