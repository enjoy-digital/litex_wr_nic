#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os

from pathlib import Path

from migen import *
from migen.genlib.cdc import MultiReg
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import CSRField, CSRStatus

from litex.soc.cores.code_8b10b import Encoder, Decoder

# Sources ------------------------------------------------------------------------------------------

def phy8_sources(platform):
    """Portable WR logic; keep SerDes, clocking and pads out of VHDL."""
    from litex_wr_nic.gateware.wr_common import (
        wr_core_init,
        wr_core_files,
        patch_wr_subsystem_mux_class,
        patch_wr_clock_monitor_presc_cdc,
        patch_wr_external_cpu_memory,
        patch_wr_diags_control_word,
        _replace_once,
    )
    wr_core_init()
    patch_wr_subsystem_mux_class()
    patch_wr_clock_monitor_presc_cdc()
    patch_wr_external_cpu_memory()
    patch_wr_diags_control_word()
    overrides = {
        # GHDL treats unspecified std_logic states as a latch. The eight
        # binary selector values are already exhaustive.
        "wr-cores/ip_cores/general-cores/modules/wishbone/wb_uart/uart_async_tx.vhd": (
            "      when others => null;", "      when others => muxbit <= '0';"),
        # GW5 DPB does not support read-first. The diagnostic RAM explicitly
        # allows don't-care read-during-write data, so write-first is valid.
        "wr-cores/modules/wrc_core/wrc_diags_dpram.vhd": (
            'g_addr_conflict_resolution => "dont_care"',
            'g_addr_conflict_resolution => "write_first"'),
    }
    sources = []
    for filename in wr_core_files:
        if any(part in filename for part in ("/platform/", "/board/litex_wr_nic/", "/ps_gen/")):
            continue
        # The Altera variant is ordinary inferred RTL; the Xilinx variant
        # explicitly instantiates SRLC32E for short shift registers.
        filename = filename.replace(
            "genrams/xilinx/gc_shiftreg.vhd", "genrams/altera/gc_shiftreg.vhd")
        if filename.endswith(".v"):
            platform.add_source(filename)
        else:
            if filename in overrides:
                # Keep vendor-specific changes out of the shared upstream tree.
                copy = Path("wr-cores/.litex-phy8") / Path(filename).name
                copy.parent.mkdir(parents=True, exist_ok=True)
                copy.write_bytes(Path(filename).read_bytes())
                _replace_once(copy, *overrides[filename])
                filename = str(copy)
            sources.append(os.path.abspath(filename))
    for filename in (
        "wr_irig/wr_irig_master.vhd",
        "wr_nmea/nmea_master_regs.vhd",
        "wr_nmea/wr_nmea_master.vhd",
        "wr_nmea/xwr_nmea_master.vhd",
        "wr_auxclk_gen/auxclk_regs.vhd",
        "wr_auxclk_gen/xwr_auxclk_gen.vhd",
    ):
        sources.append(os.path.abspath("wr-cores/modules/" + filename))
    sources.append(os.path.join(
        os.path.dirname(__file__), "wr-cores/board/litex_wr_nic/xwrc_litex_phy8.vhd"))
    return sources

# Gowin WR PHY -------------------------------------------------------------------------------------

class GW5WRPHY(LiteXModule):
    """WR's 8-bit PCS interface on LiteEth's raw Gowin SerDes.

    This is an initial link/clock bring-up interface. Hardware comma alignment
    and the RX FIFO do not yet provide a calibrated bitslide/latency value.
    """
    def __init__(self, platform, lane=0):
        from liteeth.phy.gw5_1000basex import GW5SerDes

        self.reset        = Signal()
        self.loopback     = Signal()
        self.tx_data      = Signal(8)
        self.tx_k         = Signal()
        self.tx_disparity = Signal()
        self.tx_error     = Signal()
        self.rx_data      = Signal(8)
        self.rx_k         = Signal()
        self.rx_error     = Signal()
        self.rx_bitslide  = Signal(4)
        self.ready        = Signal()

        self.cd_wr_phy_tx    = ClockDomain()
        self.cd_wr_phy_rx    = ClockDomain()
        self.cd_wr_phy_ready = ClockDomain()

        self.status = CSRStatus(fields=[
            CSRField("pll_lock", description="TX PLL locked."),
            CSRField("cdr_lock", description="RX CDR locked."),
            CSRField("aligned",  description="RX comma alignment acquired."),
            CSRField("rx_valid", description="Raw RX symbol valid."),
            CSRField("ready",    description="WR PHY ready; synchronized to the RX clock."),
        ])

        # # #

        # SerDes clocks/resets.
        self.serdes = serdes = GW5SerDes(platform, lane=lane)
        self.tx_clk   = serdes.tx_clk
        self.rx_clk   = serdes.rx_clk
        self.pll_lock = serdes.pll_lock
        self.comb += [
            serdes.reset.eq(self.reset | ResetSignal("sys")),
            self.cd_wr_phy_tx.clk.eq(self.tx_clk),
            self.cd_wr_phy_rx.clk.eq(self.rx_clk),
            self.cd_wr_phy_ready.clk.eq(self.rx_clk),
            self.ready.eq(~self.cd_wr_phy_ready.rst),
            self.tx_error.eq(0),
            self.rx_bitslide.eq(0),
        ]
        self.specials += [
            AsyncResetSynchronizer(self.cd_wr_phy_tx, serdes.reset | ~serdes.pll_lock),
            AsyncResetSynchronizer(self.cd_wr_phy_rx, serdes.reset | ~serdes.cdr_lock),
            # WR uses ready to release its RX logic. Deassert even if the
            # recovered clock stops, and release only on recovered RX edges.
            AsyncResetSynchronizer(self.cd_wr_phy_ready,
                serdes.reset | ~serdes.pll_lock | ~serdes.cdr_lock | ~serdes.aligned),
        ]

        # 8b/10b coding.
        self.encoder = encoder = ClockDomainsRenamer("wr_phy_tx")(Encoder(lsb_first=True))
        self.decoder = decoder = ClockDomainsRenamer("wr_phy_rx")(Decoder(lsb_first=True))
        self.comb += [
            encoder.d[0].eq(self.tx_data),
            encoder.k[0].eq(self.tx_k),
        ]
        self.sync.wr_phy_tx += [
            serdes.tx_data.eq(encoder.output[0]),
            self.tx_disparity.eq(encoder.disparity[0]),
        ]
        # Keep every symbol, including idle/configuration words. WR owns the PCS,
        # autonegotiation and timestamps; no LiteEth MAC or packet buffer intervenes.
        rx_valid = Signal(2)
        self.sync.wr_phy_rx += [
            decoder.input.eq(serdes.rx_data[:10]),
            rx_valid.eq(Cat(serdes.rx_valid & serdes.aligned, rx_valid[0])),
        ]
        self.comb += [
            self.rx_data.eq(decoder.d),
            self.rx_k.eq(decoder.k),
            self.rx_error.eq(decoder.invalid | ~rx_valid[1]),
        ]

        # Synchronize each diagnostic flag into the CSR clock domain.
        self.specials += MultiReg(
            Cat(serdes.pll_lock, serdes.cdr_lock, serdes.aligned, serdes.rx_valid, self.ready),
            Cat(self.status.fields.pll_lock, self.status.fields.cdr_lock,
                self.status.fields.aligned, self.status.fields.rx_valid, self.status.fields.ready),
        )
