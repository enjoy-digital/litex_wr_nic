from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

# Phase Shift Generator -----------------------------------------------------------------------------

class TXPITunner(LiteXModule):
    def __init__(self, ctrl_size=16):
        """
        Phase Shift Generator module for generating phase shift control signals.

        Parameters:
        -----------
        ctrl_size : int
            Size of the control data signal (default: 16 bits).
        """

        # Control Interface.
        self.ctrl_data = Signal(ctrl_size)
        self.ctrl_load = Signal()

        # TXPI Interface.
        self.txpippmstepsize     = Signal(5)

        # # #

        psincdec = Signal() # wether we increment or decrement the phase
        speed = Signal(4) # phase shift size

        # Command is ctrl_data sampled when ctrl_load is asserted
        neutral = 1 << (ctrl_size - 1)
        cmd = Signal(ctrl_size, reset=neutral) 
        self.sync += If(self.ctrl_load,
                cmd.eq(self.ctrl_data)
        )

        # buffer reg needed between combinatorial logic and GTPE2 port
        self.sync += self.txpippmstepsize.eq(Cat(speed, psincdec))

        # Absolute command and phase shift direction.
        cmd_abs = Signal(ctrl_size)
        self.sync += If(cmd >= neutral,
                psincdec.eq(0), # decrement the phase -> speed up
                cmd_abs.eq(cmd - neutral)
            ).Else(
                psincdec.eq(1), # increment the phase -> slow down
                cmd_abs.eq(neutral - cmd)
            )

        # Sigma delta to dither the ctrl_size bits into the 4 bits of txpippmstepsize
        acc = Signal(ctrl_size)
        cnt = Signal(1)
        self.sync += [
            If(cnt == 0, # the GTPE2 needs the stepsize to be stable for 2 clock cycles (TXPI_SYNFREQ_PPM[2:0] + 1 cycles)
                acc.eq(acc + cmd_abs - (acc & (0b1111 << (ctrl_size - 4)))),
                speed.eq(acc >> (ctrl_size - 4)),
               ),
            cnt.eq(cnt + 1),
        ]
