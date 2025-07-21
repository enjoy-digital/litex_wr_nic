from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

# Phase Shift Generator -----------------------------------------------------------------------------

class PSGen(LiteXModule):
    def __init__(self, cd_psclk, cd_sys, ctrl_size=16, div_n=0):
        """
        Phase Shift Generator module for generating phase shift control signals.

        Parameters:
        -----------
        cd_psclk : str
            Clock domain for phase shift control signals.
        cd_sys : str
            System clock domain.
        ctrl_size : int
            Size of the control data signal (default: 16 bits).
        div_n : int
            Division factor to scale the tuning range and resolution (default: 0).
        """

        # Control Interface.
        self.ctrl_data = Signal(ctrl_size)
        self.ctrl_load = Signal()

        # Phase Shift Interface.
        self.psen     = Signal()
        self.psincdec = Signal()

        # # #

        # Scaled command size: Extends control size by div_n for finer resolution.
        self.scaled_cmd_size = ctrl_size + div_n
        self.scaled_cmd      = Signal(self.scaled_cmd_size)

        # Neutral points for control and scaled commands.
        neutral        = 1 << (ctrl_size - 1)
        scaled_neutral = 1 << (self.scaled_cmd_size - 1)
        center_cmd     = scaled_neutral

        # System clock domain: Load and scale control data.
        sync_sys = getattr(self.sync, cd_sys)
        sync_sys += If(self.ctrl_load,
                self.scaled_cmd.eq(center_cmd + self.ctrl_data - neutral)
        )

        # Absolute scaled command for phase shift direction.
        self.scaled_cmd_abs = Signal(self.scaled_cmd_size - 1)
        sync_sys += If(self.scaled_cmd >= scaled_neutral,
            self.psincdec.eq(1),
            self.scaled_cmd_abs.eq(self.scaled_cmd - scaled_neutral)
        ).Else(
            self.psincdec.eq(0),
            self.scaled_cmd_abs.eq(scaled_neutral - self.scaled_cmd)
        )

        # Clock domain crossing: Transfer absolute command to psclk domain.
        self.scaled_cmd_psclk = Signal(self.scaled_cmd_size - 1)
        self.specials += MultiReg(
            i       = self.scaled_cmd_abs,
            o       = self.scaled_cmd_psclk,
            odomain = cd_psclk,
        )

        # Phase shift clock domain: Accumulate and generate phase shift enable.
        sync_ps = getattr(self.sync, cd_psclk)
        self.acc_size = self.scaled_cmd_size + 3
        self.acc      = Signal(self.acc_size)
        self.acc_old  = Signal(self.acc_size)
        sync_ps += [
            self.acc.eq(self.acc + self.scaled_cmd_psclk),
            self.acc_old.eq(self.acc),
            self.psen.eq(self.acc_old > self.acc),
        ]
