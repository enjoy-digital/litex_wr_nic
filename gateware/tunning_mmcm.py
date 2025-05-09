from migen import *

from litex.gen import *
from litex.soc.cores.clock import S7PLL, S7MMCM

class TunningMMCM(S7MMCM):
    def __init__(self, cd_psclk, cd_sys, ctrl_size=16, mult=7, div=16, speedgrade=-3):
        self.ctrl_data = Signal(ctrl_size)
        self.ctrl_load = Signal()
        super().__init__(speedgrade)

        self.expose_dps(cd_psclk, with_csr=False)
        self.params.update(
            p_CLKOUT0_USE_FINE_PS = "TRUE",
            p_CLKOUT1_USE_FINE_PS = "TRUE",
            p_CLKOUT2_USE_FINE_PS = "TRUE",
            p_CLKOUT3_USE_FINE_PS = "TRUE",
            p_CLKOUT4_USE_FINE_PS = "TRUE",
            p_CLKOUT5_USE_FINE_PS = "TRUE",
            p_CLKOUT6_USE_FINE_PS = "TRUE",
        )

        self.ps_gen = Instance('ps_gen',
            p_WIDTH       = ctrl_size,
            p_DIV         = div,
            p_MULT        = mult,

            i_pswidth     = self.ctrl_data,
            i_pswidth_set = self.ctrl_load,
            i_pswidth_clk = ClockSignal(cd_sys),

            i_psclk       = ClockSignal(cd_psclk),
            i_psdone      = self.psdone,
            o_psen        = self.psen,
            o_psincdec    = self.psincdec,
        )
