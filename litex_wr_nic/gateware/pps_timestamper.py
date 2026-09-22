#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import CSR, CSRField, CSRStatus, CSRStorage

# PPS Timestamper ----------------------------------------------------------------------------------

class PPSTimestamper(LiteXModule):
    """Timestamp the rising edges of a PPS input against the WR time.

    ``samples`` holds ``len(samples)`` equally spaced samples of the input
    taken during the previous cycle of ``cd``, earliest in bit 0, and must be
    stable around that clock's active edge (a sampler phase-locked to ``cd``).
    Each rising edge latches the WR seconds/cycles of the cycle in which it
    was seen and the index of its first high sample, so the position within
    the cycle is known to one sample period. ``event`` pulses in ``cd`` on
    every timestamped edge; ``count`` counts them. A fine delay applied to the
    input before sampling, stepped by the host, refines the index further.
    """
    def __init__(self, samples, tm_seconds, tm_cycles, tm_valid, cd="wr"):
        self.event = Signal()
        self.count = Signal(32)
        self.clear = Signal() # Clear the counter, in the CSR clock domain.

        self._control = CSRStorage(fields=[
            CSRField("clear", size=1, pulse=True, description="Clear the event counter."),
        ])
        self._count   = CSRStatus(32, description="Timestamped PPS rising edges since clear.")
        self._seconds = CSRStatus(40, description="WR seconds of the latest edge.")
        self._cycles  = CSRStatus(28, description="WR cycles (reference clock periods) of the latest edge.")
        self._fine    = CSRStatus(fields=[
            CSRField("index", size=max(1, (len(samples) - 1).bit_length()),
                description="Index of the first high sample within the cycle, earliest is 0."),
            CSRField("valid", size=1, offset=8, description="WR time was valid for the latest edge."),
        ])
        self._samples = CSRStatus(len(samples),
            description="Latest raw samples of the input, earliest in bit 0.")

        # # #

        width = len(samples)
        last  = Signal()
        rises = Signal(width)
        index = Signal(max=max(width, 2)) # A single sample per cycle: index 0.
        sync  = getattr(self.sync, cd)
        # The sample before each one: the previous cycle's last sample, then
        # the samples of this cycle. A one-sample input has no earlier sample.
        previous = Cat(last, samples[:-1]) if width > 1 else last
        self.comb += rises.eq(samples & ~previous)
        # Priority encoder: the earliest rising sample of the cycle.
        for i in reversed(range(width)):
            self.comb += If(rises[i], index.eq(i))
        sync += [
            last.eq(samples[-1]),
            self.event.eq(rises != 0),
            If(rises != 0,
                self._seconds.status.eq(tm_seconds),
                self._cycles.status.eq(tm_cycles),
                self._fine.fields.index.eq(index),
                self._fine.fields.valid.eq(tm_valid),
                self.count.eq(self.count + 1),
            ),
        ]
        self.clear_sync = PulseSynchronizer("sys", cd)
        self.comb += self.clear_sync.i.eq(self.clear | self._control.fields.clear)
        sync += If(self.clear_sync.o, self.count.eq(0))
        self.specials += MultiReg(self.count, self._count.status)
        # The raw samples show the line state between edges: a floating or
        # idle input reads all-ones or all-zeros, so the path is observable
        # without a PPS source.
        self.specials += MultiReg(samples, self._samples.status)

# Gowin GW5A Oversampled Input ---------------------------------------------------------------------

class GW5OversampledInput(LiteXModule):
    """Eight samples per reference cycle of an input pin on GW5A.

    A PLL locked to ``clk`` (the 125 MHz WR reference) generates a 500 MHz
    sampling clock and a 125 MHz word clock 180 degrees from the reference,
    so the IDES8 word crosses into the reference domain with half a period of
    margin. The pin goes through an IODELAY of about 12.5 ps per tap before
    the deserializer; ``delay`` selects the tap from the host so the edge
    position can be refined below the 1 ns sample spacing by scanning.
    """
    def __init__(self, platform, pad, clk, clk_freq=125e6, oversampling=8):
        if oversampling != 8:
            raise ValueError("The GW5A sampler uses IDES8: eight samples per cycle.")
        self.samples = Signal(oversampling) # cd_word: earliest sample first.

        self._delay = CSRStorage(fields=[
            CSRField("taps", size=8, description="IODELAY taps (~12.5 ps each) before the sampler."),
            CSRField("load", size=1, offset=8, pulse=True, description="Apply the taps."),
        ])

        # # #

        from litex.soc.cores.clock.gowin_gw5a import GW5APLL

        self.cd_fast = ClockDomain()
        self.cd_word = ClockDomain()
        self.pll = pll = GW5APLL(device=platform.device, devicename=platform.devicename)
        pll.register_clkin(clk, clk_freq)
        pll.create_clkout(self.cd_fast, 4*clk_freq, margin=0)
        pll.create_clkout(self.cd_word, clk_freq, phase=180, margin=0)

        # IODELAY: dynamic taps are loaded on a VALUE edge while SDTAP is low.
        delayed = Signal()
        load    = Signal()
        self.sync += load.eq(self._delay.fields.load)
        self.specials += Instance("IODELAY",
            p_C_STATIC_DLY = 0,
            p_DYN_DLY_EN   = "TRUE",
            p_ADAPT_EN     = "FALSE",
            i_DI           = pad,
            i_SDTAP        = 0,
            i_VALUE        = load,
            i_DLYSTEP      = self._delay.fields.taps,
            o_DO           = delayed,
            o_DF           = Open(),
        )

        # IDES8: both edges of the 500 MHz clock, framed by the word clock.
        # Release its reset synchronously to the word clock so the framing of
        # the eight samples is the same after every reset.
        reset = Signal(reset=1)
        self.sync.word += reset.eq(~pll.locked)
        samples = [Signal() for _ in range(oversampling)]
        self.specials += Instance("IDES8",
            i_D     = delayed,
            i_FCLK  = ClockSignal("fast"),
            i_PCLK  = ClockSignal("word"),
            i_CALIB = 0,
            i_RESET = reset,
            **{f"o_Q{i}": samples[i] for i in range(oversampling)},
        )
        self.comb += self.samples.eq(Cat(*samples))
