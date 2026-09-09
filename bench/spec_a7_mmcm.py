#!/usr/bin/env python3
#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Temporary SPEC-A7 image for measuring WRMMCMBackend against real MMCMs."""

import argparse

from migen import *
from migen.genlib.cdc import BusSynchronizer, GrayCounter, GrayDecoder, MultiReg, PulseSynchronizer

from litex.gen import LiteXModule

from litex.soc.cores.clock import S7MMCM, S7PLL
from litex.soc.integration.builder import Builder
from litex.soc.integration.soc_core import SoCMini
from litex.soc.interconnect.csr import CSRStatus, CSRStorage

from spec_a7_platform import Platform
from litex_wr_nic.gateware.wr_clock import WRMMCMBackend

# Clocking -----------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform):
        self.cd_sys      = ClockDomain("sys")
        self.cd_ps       = ClockDomain("ps")
        self.cd_input100 = ClockDomain("input100")
        self.cd_command  = ClockDomain("command_raw", reset_less=True)
        self.pll = pll = S7PLL(speedgrade=-2)
        self.comb += platform.request("clk125m_oe").eq(1)
        pll.register_clkin(platform.request("clk125m"), 125e6)
        pll.create_clkout(self.cd_sys,      62.5e6, margin=0)
        pll.create_clkout(self.cd_ps,        200e6, margin=0)
        pll.create_clkout(self.cd_input100,  100e6, margin=0)
        # Drive the independently gated command clocks directly from a PLL
        # output. A single BUFG cannot legally cascade into four BUFGCEs.
        pll.create_clkout(self.cd_command, 62.5e6, margin=0, with_reset=False, buf=None)

# Hardware Qualification Channel ------------------------------------------------------------------

class MMCMChannel(LiteXModule):
    def __init__(self, name, input_cd, input_freq, output_freq, gate_cycles=2**24):
        self._code        = CSRStorage(16, reset=32768)
        self._enable      = CSRStorage()
        self._period      = CSRStorage(16, reset=1)
        self._reset       = CSRStorage(reset=1)
        self._pause       = CSRStorage()
        self._drop_done   = CSRStorage()
        self._measure     = CSRStorage()
        self._locked      = CSRStatus()
        self._errors      = CSRStatus(32)
        self._gate_cycles = CSRStatus(32, reset=gate_cycles)
        self._frequency   = CSRStatus(32, reset=int(output_freq))

        # # #

        # The command clock can be stopped without stopping host access or
        # PSCLK. All host settings are stable before resuming this clock.
        self.cd_command = ClockDomain(f"{name}_command")
        self.specials += Instance("BUFGCE",
            i_I  = ClockSignal("command_raw"),
            i_CE = ~self._pause.storage,
            o_O  = self.cd_command.clk,
        )
        self.comb += self.cd_command.rst.eq(ResetSignal("sys") | self._reset.storage)

        # Respect the SPEC-A7's -2 speed grade. The selected VCO is exported
        # for the host's independent output-frequency prediction.
        self.cd_output = ClockDomain(f"{name}_output")
        self.mmcm = mmcm = S7MMCM(speedgrade=-2, fractional=False)
        mmcm.register_clkin(ClockSignal(input_cd), input_freq)
        mmcm.create_clkout(self.cd_output, output_freq, margin=0)
        mmcm.expose_dps("ps", with_csr=False)
        mmcm.params["p_CLKOUT0_USE_FINE_PS"] = "TRUE"
        mmcm_reset = Signal()
        self.specials += MultiReg(self._reset.storage | ResetSignal("sys"), mmcm_reset, input_cd)
        self.comb += mmcm.reset.eq(mmcm_reset)
        self._vco = CSRStatus(32, reset=int(mmcm.compute_config()["vco"]))

        self.backend = backend = WRMMCMBackend("ps", self.cd_command.name)
        period = Signal(16)
        sync = getattr(self.sync, self.cd_command.name)
        sync += [
            backend.command.load.eq(0),
            If(self._enable.storage,
                If(period == 0,
                    backend.command.data.eq(self._code.storage),
                    backend.command.load.eq(1),
                    period.eq(self._period.storage - 1),
                ).Else(
                    period.eq(period - 1),
                ),
            ).Else(
                period.eq(0),
            ),
        ]
        drop_done = Signal()
        self.specials += MultiReg(self._drop_done.storage, drop_done, "ps")
        self.comb += [
            mmcm.psen.eq(backend.psen),
            mmcm.psincdec.eq(backend.psincdec),
            backend.psdone.eq(mmcm.psdone & ~drop_done),
        ]

        # Count actual output-clock edges and transfer the Gray count to
        # PSCLK. Only the measurement interval, not the phase accumulator,
        # participates in the independent frequency measurement.
        self.counter = counter = ClockDomainsRenamer(self.cd_output.name)(GrayCounter(32))
        self.decoder = decoder = ClockDomainsRenamer("ps")(GrayDecoder(32))
        self.comb += counter.ce.eq(1)
        self.specials += MultiReg(counter.q, decoder.i, "ps")

        pending    = Signal()
        direction  = Signal()
        previous   = Signal()
        age        = Signal(16)
        issued     = Signal(32)
        completed  = Signal(32)
        phase      = Signal(32)
        errors     = Signal(32)
        reset_ps   = Signal()
        locked_ps  = Signal()
        had_lock   = Signal()
        self.specials += [
            MultiReg(self._reset.storage, reset_ps, "ps"),
            MultiReg(mmcm.locked, locked_ps, "ps"),
        ]
        self.sync.ps += [
            previous.eq(backend.psen),
            If(reset_ps,
                pending.eq(0),
                previous.eq(0),
                age.eq(0),
                issued.eq(0),
                completed.eq(0),
                phase.eq(0),
                errors.eq(0),
                had_lock.eq(0),
            ).Elif(locked_ps,
                had_lock.eq(1),
                If(pending, age.eq(age + 1)),
                If(backend.psen,
                    pending.eq(1),
                    direction.eq(backend.psincdec),
                    age.eq(0),
                    issued.eq(issued + 1),
                    If(pending, errors[0].eq(1)),
                    If(previous, errors[1].eq(1)),
                ),
                If(pending & (backend.psincdec != direction), errors[2].eq(1)),
                If(mmcm.psdone,
                    pending.eq(0),
                    completed.eq(completed + 1),
                    If(direction,
                        phase.eq(phase + 1),
                    ).Else(
                        phase.eq(phase - 1),
                    ),
                    If(~pending, errors[3].eq(1)),
                    If(age != 11, errors[4].eq(1)),
                ),
            ).Elif(had_lock,
                errors[5].eq(1),
            ),
        ]
        self.live_cdc = live_cdc = BusSynchronizer(33, "ps", "sys")
        self.comb += [
            live_cdc.i.eq(Cat(errors, locked_ps)),
            self._errors.status.eq(live_cdc.o[:32]),
            self._locked.status.eq(live_cdc.o[32]),
        ]

        # Freeze all results at the same PSCLK edge and publish a new sample
        # identifier with them. Host reads cannot mix two measurement windows.
        self.trigger = trigger = PulseSynchronizer("sys", "ps")
        self.comb += trigger.i.eq(self._measure.re)
        measuring = Signal()
        timer     = Signal(max=gate_cycles)
        sequence  = Signal(32)
        signals   = dict(edges=decoder.o, phase=phase, issued=issued,
            completed=completed, steps=backend.steps, errors=errors)
        starts    = {name: Signal(32) for name in signals}
        samples   = {name: Signal(32) for name in signals}
        self.sync.ps += If(reset_ps,
            measuring.eq(0),
        ).Elif(trigger.o & ~measuring,
            measuring.eq(1),
            timer.eq(0),
            *[starts[name].eq(signal) for name, signal in signals.items()],
        ).Elif(measuring,
            If(timer == gate_cycles - 1,
                measuring.eq(0),
                sequence.eq(sequence + 1),
                *[samples[name].eq(signal - starts[name]) for name, signal in signals.items()],
                samples["errors"].eq(errors),
            ).Else(
                timer.eq(timer + 1),
            ),
        )
        samples["id"] = sequence
        self.sample_cdc = sample_cdc = BusSynchronizer(32*len(samples), "ps", "sys")
        self.comb += sample_cdc.i.eq(Cat(*samples.values()))
        for index, name in enumerate(samples):
            csr = CSRStatus(32, name=f"sample_{name}")
            setattr(self, f"_sample_{name}", csr)
            self.comb += csr.status.eq(sample_cdc.o[32*index:32*(index + 1)])

# SoC ----------------------------------------------------------------------------------------------

class MMCMTestSoC(SoCMini):
    def __init__(self):
        platform = Platform(variant="xc7a50t")
        self.crg = CRG(platform)
        SoCMini.__init__(self, platform, clk_freq=62.5e6,
            ident="WR MMCM hardware qualification", ident_version=False)
        self.add_jtagbone()
        platform.add_period_constraint(self.jtagbone_phy.cd_jtag.clk, 1e9/20e6)
        platform.add_false_path_constraints(self.jtagbone_phy.cd_jtag.clk, self.crg.cd_sys.clk)
        for input_freq, input_cd in [(100e6, "input100"), (200e6, "ps")]:
            for kind, output_freq in [("ref", 125e6), ("dmtd", 62.5e6)]:
                name = f"{kind}{int(input_freq/1e6)}"
                channel = MMCMChannel(name, input_cd, input_freq, output_freq)
                setattr(self, name, channel)
                platform.add_false_path_constraints(channel.cd_command.clk, self.crg.cd_ps.clk)
                # The tuned clocks can acquire any phase relative to the
                # free-running clocks. Their only crossings use Gray counters.
                platform.add_false_path_constraints(channel.cd_output.clk, self.crg.cd_ps.clk)
                platform.add_false_path_constraints(channel.cd_output.clk, self.crg.cd_sys.clk)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build",      action="store_true",                    help="Build the temporary SRAM image.")
    parser.add_argument("--output-dir", default="build/spec_a7_mmcm",           help="Build output directory.")
    args = parser.parse_args()
    soc = MMCMTestSoC()
    builder = Builder(soc, output_dir=args.output_dir, compile_software=False,
        csr_csv=f"{args.output_dir}/csr.csv")
    builder.build(run=args.build, build_name="spec_a7_mmcm")


if __name__ == "__main__":
    main()
