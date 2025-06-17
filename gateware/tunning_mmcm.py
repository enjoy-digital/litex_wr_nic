from migen import *

from litex.gen import *
from litex.soc.cores.clock import S7MMCM

class TunningMMCM(S7MMCM):
    def __init__(self, speedgrade=-3):
        self.ctrl_data = Signal(16)
        self.ctrl_load = Signal()
        super().__init__(speedgrade)

        self.expose_dps("clk200", with_csr=False)
        self.params.update(p_CLKOUT0_USE_FINE_PS="TRUE")

        self.ps_gen = Instance("ps_gen",
            p_WIDTH       = 16,
            p_DIV         = 16,
            p_MULT        = 7,

            i_pswidth     = self.ctrl_data,
            i_pswidth_set = self.ctrl_load,
            i_pswidth_clk = ClockSignal("wr"),

            i_psclk       = ClockSignal("clk200"),
            i_psdone      = self.psdone,
            o_psen        = self.psen,
            o_psincdec    = self.psincdec,
        )
