from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

class PLL_DMTD(LiteXModule):
    """Takes a clock signal at 62.5 MHz or 125 MHz
    and produce a 2^14 / (2^14 + 1) * 62.5 MHz clock signal.
    - clk_in should be the white rabbit syntonized clock, and
    - clk_dmtd can be used as the white rabbit DMTD clock.
    The white rabbit firmware needs to be compiled with CONFIG_IGNORE_HPLL when using this module.
    """
    def __init__(self, clk_in_f=62.5e6):
        self.clk_in = Signal()
        self.clk_dmtd = Signal()

        self.mmcm1_feedback = Signal()
        self.mmcm1_out = Signal()
        self.mmcm1 = Instance(
                'MMCME2_ADV',
                p_COMPENSATION = 'INTERNAL',
                p_CLKIN1_PERIOD = 1e9/clk_in_f,
                p_CLKFBOUT_MULT_F = 16 * 62.5e6 / clk_in_f,
                p_CLKOUT0_DIVIDE_F = 14.125,

                i_CLKIN1 = self.clk_in,
                o_CLKOUT0 = self.mmcm1_out,

                i_CLKFBIN = self.mmcm1_feedback,
                o_CLKFBOUT = self.mmcm1_feedback,
                i_CLKINSEL = 1,
                )

        self.mmcm2_feedback = Signal()
        self.mmcm2 = Instance(
                'MMCME2_ADV',
                p_COMPENSATION = 'INTERNAL',
                p_CLKIN1_PERIOD = 1e9/70.8e6,
                p_CLKFBOUT_MULT_F = 16,
                p_CLKOUT0_DIVIDE_F = 18.125,

                i_CLKIN1 = self.mmcm1_out,
                o_CLKOUT0 = self.clk_dmtd,

                i_CLKFBIN = self.mmcm2_feedback,
                o_CLKFBOUT = self.mmcm2_feedback,
                i_CLKINSEL = 1,
                )
