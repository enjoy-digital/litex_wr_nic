#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2025 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litex.gen import *

from litex.soc.interconnect.csr import *

# MMCM Freq Controller -----------------------------------------------------------------------------

class MMCMFreqController(LiteXModule):
    """
    MMCMFreqController Core.

    Provides a frequency control mechanism for an MMCM by dynamically adjusting the phase shift.
    This module mimics VCXO-like frequency tuning using the MMCM's Dynamic Phase Shift (DPS) feature.

    Features:
    - Configurable control width (default 16-bit: 0 = max slow, neutral = midpoint, max = max speed).
    - Rate control via accumulator with configurable scaling factor.
    - FSM for precise timing of phase shift operations.
    - Optional CSR interface for configuration.
    """
    def __init__(self, mmcm, clk_freq, phase_shift_cycles=13, control_width=16, with_csr=True):
        self.control = Signal(control_width)

        # # #

        # Neutral Value.
        # --------------
        neutral = 2**(control_width - 1)

        # Direction Calculation.
        # ----------------------
        self.comb += mmcm.psincdec.eq(self.control > neutral)

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
        # occur approximately every `phase_shift_cycles` clock cycles.
        k        = int((2**32 / phase_shift_cycles) / (neutral - 1))
        acc      = Signal(32)
        acc_last = Signal(32)
        self.overflow = overflow = Signal()
        self.sync += acc.eq(acc + (magnitude * k))
        self.sync += acc_last.eq(acc)
        self.comb += overflow.eq(acc < acc_last)

        # Phase Shift Control FSM.
        # ------------------------
        self.fsm = FSM(reset_state="IDLE")
        self.fsm.act("IDLE",
            If(overflow,
                mmcm.psen.eq(1),
                NextState("WAIT")
            )
        )
        self.fsm.act("WAIT",
            If(mmcm.psdone,
                NextState("IDLE")
            )
        )

        # CSR.
        # ----
        if with_csr:
            self.add_csr(control_width, neutral)

    def add_csr(self, control_width, neutral):
        self._control = CSRStorage(control_width, description="Frequency control: 0=max slow, midpoint=neutral, max=max speed", reset=neutral)
        self.comb += self.control.eq(self._control.storage)
