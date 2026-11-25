#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""LiteEth 1000BASE-X link check on Tang Mega 138K Pro (no White Rabbit).

Bring-up companion to tang_mega_138k_pro_wr.py: the same GW5 SerDes lane and
dock reference clock (MS5351 PLL0, 100 MHz on Q1 REFCLK1), but with LiteEth's
own 1000BASE-X PCS and autonegotiation instead of the WR endpoint. It checks
the SFP physical layer against any 1000BASE-X peer (e.g. acorn_liteeth_nic.py)
without WR firmware, clock actuators or a host Ethernet interface.

Host access is UARTBone on the debugger's second UART interface:

    litex_server --uart --uart-port /dev/serial/by-id/FPGA_UART --uart-baudrate 115200

CSRs: ethphy_status (pll_lock, cdr_lock, aligned, link_up), ethphy_pcs_status
(link_up, is_sgmii, config_reg = link partner ability word),
link_up_events/link_down_events and tx/rx_clk_freq_value (SerDes clocks in Hz,
about 125 MHz). LEDs: 0 = PLL lock, 1 = CDR lock, 2 = comma alignment,
3 = PCS link up.
"""

import argparse

from migen import *
from migen.genlib.cdc import MultiReg
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import CSRStatus
from litex.soc.cores.freqmeter import FreqMeter
from litex.soc.integration.soc_core import SoCMini
from litex.soc.integration.builder import Builder

from litex_boards.platforms import sipeed_tang_mega_138k_pro

from liteeth.phy.gw5_1000basex import GW5_1000BASEX

# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform):
        self.cd_sys = ClockDomain()
        self.cd_por = ClockDomain(reset_less=True)

        # # #

        # Power-on reset, then the 50 MHz oscillator directly as the system clock.
        clk50 = platform.request("clk50")
        por   = Signal(16, reset=2**16 - 1)
        self.comb += [
            self.cd_sys.clk.eq(clk50),
            self.cd_por.clk.eq(clk50),
        ]
        self.sync.por += If(por != 0, por.eq(por - 1))
        self.specials += AsyncResetSynchronizer(self.cd_sys, por != 0)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCMini):
    def __init__(self, sfp=0):
        platform     = sipeed_tang_mega_138k_pro.Platform()
        sys_clk_freq = 50e6

        # CRG / SoC --------------------------------------------------------------------------------
        self.crg = CRG(platform)
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident=f"LiteEth 1000BASE-X link check on Tang Mega 138K Pro (SFP{sfp})")
        self.add_uartbone(baudrate=115200)

        # 1000BASE-X PHY ---------------------------------------------------------------------------
        sfp_pads = platform.request("sfp", sfp)
        self.comb += sfp_pads.tx_disable.eq(0)
        self.ethphy = GW5_1000BASEX(platform, lane=sfp)
        self.ethphy.pcs.add_csr(sys_clk_freq)
        platform.add_period_constraint(self.ethphy.cd_eth_tx.clk, 1e9/125e6)
        platform.add_period_constraint(self.ethphy.cd_eth_rx.clk, 1e9/125e6)
        # sys is the clk50 pad itself: constrain against the pad clock.
        platform.add_false_path_constraints(platform.lookup_request("clk50"),
            self.ethphy.cd_eth_tx.clk, self.ethphy.cd_eth_rx.clk)

        # Link Events ------------------------------------------------------------------------------
        link_up   = Signal()
        link_up_d = Signal()
        self.specials += MultiReg(self.ethphy.link_up, link_up)
        self.link_up_events   = CSRStatus(32, description="PCS link-up transitions since reset.")
        self.link_down_events = CSRStatus(32, description="PCS link-down transitions since reset.")
        self.sync += [
            link_up_d.eq(link_up),
            If(link_up & ~link_up_d, self.link_up_events.status.eq(self.link_up_events.status + 1)),
            If(~link_up & link_up_d, self.link_down_events.status.eq(self.link_down_events.status + 1)),
        ]

        # Diagnostics ------------------------------------------------------------------------------
        # SerDes clocks in Hz over a one-second sys window: a wrong dock reference shows here
        # even while the SerDes reports PLL/CDR lock.
        self.tx_clk_freq = FreqMeter(int(sys_clk_freq), clk=self.ethphy.cd_eth_tx.clk)
        self.rx_clk_freq = FreqMeter(int(sys_clk_freq), clk=self.ethphy.cd_eth_rx.clk)

        # Leds (active low) ------------------------------------------------------------------------
        status = Signal(4)
        self.specials += MultiReg(Cat(self.ethphy.pll_lock, self.ethphy.cdr_lock,
            self.ethphy.aligned, self.ethphy.link_up), status)
        for i in range(4):
            self.comb += platform.request("led_n", i).eq(~status[i])

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--build",      action="store_true",                 help="Build the bitstream.")
    parser.add_argument("--load",       action="store_true",                 help="Load the bitstream.")
    parser.add_argument("--sfp",        default=0, type=int, choices=[0, 1], help="SFP cage (default: 0).")
    parser.add_argument("--output-dir", default=None,                        help="Build output directory.")
    args = parser.parse_args()

    soc = BaseSoC(sfp=args.sfp)
    builder = Builder(soc, output_dir=args.output_dir,
        csr_csv=None if args.output_dir is None else f"{args.output_dir}/csr.csv")
    builder.build(run=args.build)
    if args.load:
        soc.platform.create_programmer().load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
