#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2025 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litex.gen import *

from litex.soc.interconnect.csr import *

# TXPIPPM Controller -------------------------------------------------------------------------------

class TXPIPPMController(LiteXModule):
    """
    TXPIPPMController Core.

    Provides a frequency control mechanism for GTP's TIPPM by dynamically adjusting the phase shift.
    This module mimics VCXO-like frequency tuning using the GTP's TIPPM interface.

    Features:
    - Configurable control width (default 16-bit: 0 = max slow, neutral = midpoint, max = max speed).
    - Fixed step size of ±1 for tx_pippm_stepsize, with direction based on control.
    - Rate control via accumulator with configurable scaling factor.
    - FSM for precise timing of phase shift operations.
    - Optional CSR interface for configuration.
    """
    def __init__(self, tippm_en, tippm_stepsize, config_cycles=5, control_width=16, with_csr=True):
        self.control = Signal(control_width)

        # # #

        # Neutral Value.
        # --------------
        neutral = 2**(control_width - 1)

        # Direction Calculation.
        # ----------------------
        direction = Signal()
        self.comb += direction.eq(self.control > neutral)

        # Magnitude Calculation.
        # ----------------------
        magnitude = Signal(control_width)
        self.comb += [
            If(self.control >= neutral,
                magnitude.eq(self.control - neutral)
            ).Else(
                magnitude.eq(neutral - self.control)
            )
        ]

        # Accumulator for Rate Control.
        # -----------------------------
        # k is the scaling factor that determines how quickly the accumulator overflows, triggering
        # phase shifts. It’s calculated to ensure that at maximum magnitude (neutral - 1), phase shifts
        # occur approximately every `config_cycles` clock cycles.
        k        = int((2**32 / config_cycles) / (neutral - 1))
        self.acc      = acc = Signal(32)
        self.acc_last = acc_last = Signal(32)
        self.overflow = overflow = Signal()
        self.sync += acc.eq(acc + (magnitude * k))
        self.sync += acc_last.eq(acc)
        self.comb += overflow.eq(acc < acc_last)

        # Step Size Control.
        # ------------------
        # tx_pippm_stepsize is set to +1 if direction=1 (acceleration), -1 if direction=0 (slowdown)
        self.comb += [
            If(direction,
                tippm_stepsize.eq(0b00001)
            ).Else(
                tippm_stepsize.eq(0b10001)
            )
        ]

        # Phase Shift Control FSM.
        # ------------------------
        count = Signal(8)
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(overflow,
                NextState("WAIT")
            )
        )
        fsm.act("WAIT",
            tippm_en.eq(1),
            NextValue(count, count + 1),
            If(count == (config_cycles - 2),  # Wait for config_cycles
                NextValue(count, 0),
                NextState("IDLE")
            ),
        )

        # CSR
        # ---
        if with_csr:
            self.add_csr(control_width, neutral)

    def add_csr(self, control_width, neutral):
        self._control = CSRStorage(control_width, description="Frequency control: 0=max slow, midpoint=neutral, max=max speed", reset=neutral)
        self.comb += self.control.eq(self._control.storage)
