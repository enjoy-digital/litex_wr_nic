from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

class PSGen(LiteXModule):
    def __init__(self,
                 cd_psclk,
                 cd_sys,
                 ctrl_size=16,
                 div_n=0, # divide the tunning range and multiply the resolution by 2**div_n
                 ):
        self.ctrl_data = Signal(ctrl_size)
        self.ctrl_load = Signal()

        self.psen = Signal()
        self.psincdec = Signal()

        self.scaled_cmd_size = ctrl_size + div_n

        self.scaled_cmd = Signal(self.scaled_cmd_size)
        neutral = 1 << (ctrl_size - 1)
        scaled_neutral = 1 << (self.scaled_cmd_size - 1)
        center_cmd = scaled_neutral

        sync_sys = getattr(self.sync, cd_sys)
        sync_sys += If(self.ctrl_load,
                                self.scaled_cmd.eq(center_cmd + self.ctrl_data - neutral))

        self.scaled_cmd_abs = Signal(self.scaled_cmd_size - 1)
        sync_sys += If(self.scaled_cmd >= scaled_neutral,
                                self.psincdec.eq(1),
                                self.scaled_cmd_abs.eq(self.scaled_cmd - scaled_neutral)
                         ).Else(
                                self.psincdec.eq(0),
                                self.scaled_cmd_abs.eq(scaled_neutral - self.scaled_cmd))

        self.scaled_cmd_psclk = Signal(self.scaled_cmd_size - 1)
        self.specials += MultiReg(
                i = self.scaled_cmd_abs,
                o = self.scaled_cmd_psclk,
                odomain = cd_psclk,
                )

        sync_ps = getattr(self.sync, cd_psclk)
        self.acc_size = self.scaled_cmd_size + 3
        self.acc = Signal(self.acc_size)
        self.acc_old = Signal(self.acc_size)

        sync_ps += self.acc.eq(self.acc + self.scaled_cmd_psclk)
        sync_ps += self.acc_old.eq(self.acc)
        sync_ps += self.psen.eq(self.acc_old > self.acc)
