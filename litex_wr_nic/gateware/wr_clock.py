#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import BusSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import CSRField, CSRStatus

from litex_wr_nic.gateware.wr_cdc import WRClockCrossing

# WR Tuning Command -------------------------------------------------------------------------------

class WRTuningInterface(Record):
    """Unsigned tuning code, sampled on load in the declared clock domain."""
    def __init__(self, width=16, name=None):
        Record.__init__(self, [("data", width, DIR_M_TO_S), ("load", 1, DIR_M_TO_S)], name=name)


class WRTuningCalibration(LiteXModule):
    """Calibrate a centered tuning code with signed arithmetic and saturation."""
    def __init__(self, width=16, cd="wr_sys", polarity=1, gain=1, gain_shift=0, offset=0):
        if not all(isinstance(value, int) for value in (width, polarity, gain, gain_shift, offset)):
            raise ValueError("WR tuning calibration uses integer parameters.")
        if width < 2 or polarity not in (-1, 1) or gain < 1 or gain_shift < 0:
            raise ValueError("Invalid WR tuning width, polarity or gain.")
        self.sink    = sink   = WRTuningInterface(width, name="sink")
        self.source  = source = WRTuningInterface(width, name="source")
        self.clipped = Signal()

        # # #

        neutral = 1 << (width - 1)
        maximum = (1 << width) - 1
        delta   = Signal((width + 1, True))
        product = Signal((width + gain.bit_length() + 2, True))
        result  = Signal((max(len(product), abs(offset).bit_length() + 2) + 1, True))
        self.comb += [
            delta.eq(sink.data - neutral),
            product.eq(delta * (polarity * gain)),
            result.eq(neutral + offset + (product >> gain_shift)),
        ]
        sync = getattr(self.sync, cd)
        sync += [
            source.load.eq(sink.load),
            If(sink.load,
                self.clipped.eq((result < 0) | (result > maximum)),
                If(result < 0,
                    source.data.eq(0),
                ).Elif(result > maximum,
                    source.data.eq(maximum),
                ).Else(
                    source.data.eq(result),
                ),
            ),
        ]


class WRTuningCDC(LiteXModule):
    """Coherent command FIFO with one pending, replaceable latest command.

    WR emits pulses and cannot be stalled. If the FIFO is full, a newer
    command replaces the pending command and increments superseded.
    """
    def __init__(self, width=16, cd_from="wr_sys", cd_to="sys", depth=4):
        self.sink   = sink   = WRTuningInterface(width, name="sink")
        self.source = source = WRTuningInterface(width, name="source")

        self.superseded = Signal(32)

        # # #

        self.cdc = cdc = WRClockCrossing([("data", width)], cd_from, cd_to, depth)
        self.input_cd  = cdc.input_cd
        self.output_cd = cdc.output_cd
        pending = Signal()
        latest  = Signal(width)
        self.comb += [
            cdc.sink.valid.eq(pending),
            cdc.sink.data.eq(latest),
            cdc.source.ready.eq(1),
            source.load.eq(cdc.source.valid),
            source.data.eq(cdc.source.data),
        ]
        sync = getattr(self.sync, cdc.input_cd)
        sync += [
            If(cdc.sink.ready, pending.eq(0)),
            If(sink.load,
                pending.eq(1),
                latest.eq(sink.data),
                If(pending & ~cdc.sink.ready, self.superseded.eq(self.superseded + 1)),
            ),
        ]

# WR Clock Backends -------------------------------------------------------------------------------

class WRDACBackend(LiteXModule):
    """Calibrated parallel command for an external DAC driver."""
    def __init__(self, width=16, cd="wr_sys", **calibration):
        self.calibration = WRTuningCalibration(width, cd, **calibration)
        self.command = self.calibration.sink
        self.value   = self.calibration.source.data
        self.load    = self.calibration.source.load
        self.clipped = self.calibration.clipped


class WRMMCMBackend(LiteXModule):
    """Rate accumulator with one outstanding MMCM phase shift at a time."""
    def __init__(self, cd_psclk, cd_command="wr_sys", width=16, div_n=0,
        timeout_cycles=1024, **calibration):
        if div_n < 0 or timeout_cycles < 2:
            raise ValueError("Invalid MMCM rate divider or completion timeout.")
        self.command  = WRTuningInterface(width, name="command")
        self.psen     = Signal()
        self.psincdec = Signal()
        self.psdone   = Signal()
        self.busy     = Signal()
        self.fault    = Signal()
        self.steps    = Signal(32)

        # # #

        self.cdc = cdc = WRTuningCDC(width, cd_command, cd_psclk)
        self.calibration = calibrated = WRTuningCalibration(width, cdc.input_cd, **calibration)
        self.comb += [
            self.command.connect(calibrated.sink),
            calibrated.source.connect(cdc.sink),
        ]

        # Nominal rate: |code - center| / 2**(width+div_n+3) shifts per PSCLK.
        # Preserve fractional phase across same-direction updates, otherwise
        # frequent servo commands can prevent small corrections from issuing.
        # Neutral/reversal cancels only unissued phase; an active shift finishes.
        neutral       = 1 << (width - 1)
        acc_width     = width + div_n + 3
        magnitude     = Signal(width)
        direction     = Signal()
        new_magnitude = Signal(width)
        new_direction = Signal()
        acc           = Signal(acc_width)
        total         = Signal(acc_width + 1)
        timer         = Signal(max=timeout_cycles)
        self.comb += [
            new_direction.eq(cdc.source.data < neutral),
            new_magnitude.eq(Mux(new_direction,
                neutral - cdc.source.data, cdc.source.data - neutral)),
            total.eq(acc + Mux(cdc.source.load, new_magnitude, magnitude)),
        ]
        sync = getattr(self.sync, cdc.output_cd)
        sync += [
            self.psen.eq(0),
            If(cdc.source.load,
                direction.eq(new_direction),
                magnitude.eq(new_magnitude),
            ),
            If(cdc.source.load & ((new_magnitude == 0) | (new_direction != direction)),
                acc.eq(0),
            ).Elif(~self.fault,
                # Keep a due request while busy; never overlap PSEN pulses.
                If(total[acc_width],
                    If(~self.busy & ~self.psdone,
                        acc.eq(total[:acc_width]),
                        self.psen.eq(1),
                        self.psincdec.eq(direction),
                        self.busy.eq(1),
                        timer.eq(0),
                    ).Else(
                        acc.eq((1 << acc_width) - 1),
                    ),
                ).Else(
                    acc.eq(total),
                ),
            ),
            If(self.busy,
                If(self.psdone,
                    self.busy.eq(0),
                    self.steps.eq(self.steps + 1),
                ).Elif(timer == timeout_cycles - 1,
                    self.fault.eq(1),
                ).Else(
                    timer.eq(timer + 1),
                ),
            ),
        ]

        self.status_cdc = BusSynchronizer(34, cdc.output_cd, "sys")
        self._status = CSRStatus(fields=[
            CSRField("busy",  size=1, description="An MMCM phase shift is pending."),
            CSRField("fault", size=1, description="Completion timeout; reset the backend and MMCM."),
        ])
        self._steps      = CSRStatus(32, description="Completed MMCM phase shifts, wrapping at 32 bits.")
        self._superseded = CSRStatus(32, description="Commands replaced while the tuning FIFO was full.")
        self.superseded_cdc = BusSynchronizer(32, cdc.input_cd, "sys")
        self.comb += [
            self.status_cdc.i.eq(Cat(self.busy, self.fault, self.steps)),
            self._status.fields.busy.eq(self.status_cdc.o[0]),
            self._status.fields.fault.eq(self.status_cdc.o[1]),
            self._steps.status.eq(self.status_cdc.o[2:]),
            self.superseded_cdc.i.eq(cdc.superseded),
            self._superseded.status.eq(self.superseded_cdc.o),
        ]
