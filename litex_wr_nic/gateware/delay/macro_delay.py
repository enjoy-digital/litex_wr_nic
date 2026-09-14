#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litex.gen import *

from litex.soc.interconnect.csr import *

# Macro Delay --------------------------------------------------------------------------------------

class MacroDelay(LiteXModule):
    def __init__(self, pulse_i, pulse_o, clk_domain="sys", default_delay=1, pulse_cycles=1):
        if pulse_cycles < 1:
            raise ValueError("Pulse width must be at least one clock cycle.")
        self._value = CSRStorage(32, description="Macro Delay Clk Cycles.", reset=default_delay)

        # # #

        # Sync.
        _sync = getattr(self.sync, clk_domain)

        # Delay.
        self.enable = enable = Signal()
        self.count  = count  = Signal(32)
        _sync += [
            If(pulse_i,
                enable.eq(1),
                count.eq(self._value.storage - 1),
            ).Else(
                If(count == 0,
                    enable.eq(0)
                ).Else(
                    count.eq(count - 1)
                )
            )
        ]

        # Output.
        if pulse_cycles == 1:
            self.comb += pulse_o.eq(enable & (count == 0))
        else:
            # Register the widened output so asynchronous consumers cannot see
            # counter decode glitches. Anticipate count==0 to keep the same
            # leading edge, including the one-cycle delay and retrigger cases.
            remaining = Signal(max=pulse_cycles)
            trigger = (pulse_i & (self._value.storage == 1)) | (~pulse_i & enable & (count == 1))
            _sync += If(trigger,
                pulse_o.eq(1),
                remaining.eq(pulse_cycles - 1),
            ).Elif(remaining != 0,
                pulse_o.eq(1),
                remaining.eq(remaining - 1),
            ).Else(
                pulse_o.eq(0),
            )
