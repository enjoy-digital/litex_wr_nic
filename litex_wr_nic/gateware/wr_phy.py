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

from litex.soc.interconnect.csr import CSRField, CSRStatus, CSRStorage

from litex.soc.cores.code_8b10b import Encoder, Decoder

# Portable Source Preparation ----------------------------------------------------------------------

def pps_width_replacements():
    """Track the PPS pulse-width counter's zero state in a register.

    The upstream process compares the whole 28-bit counter with zero on the
    reference clock; on GW5A that comparison does not meet 125 MHz once the
    PPS output is used. The register is set when the counter is loaded with
    zero and when it reaches one, which is the same condition one cycle
    earlier. See test/hdl/wr_pps_width_tb.vhd for the equivalence check.
    """
    return [
        ("  signal width_cntr : unsigned(27 downto 0);",
         "  signal width_cntr : unsigned(27 downto 0);\n"
         "  signal width_zero : std_logic;"),
        ("        width_cntr  <= (others => '0');",
         "        width_cntr  <= (others => '0');\n"
         "        width_zero <= '1';"),
        ("          width_cntr  <= unsigned(ppsg_cr_pwidth);",
         "          width_cntr  <= unsigned(ppsg_cr_pwidth);\n"
         "          if unsigned(ppsg_cr_pwidth) = 0 then\n"
         "            width_zero <= '1';\n"
         "          else\n"
         "            width_zero <= '0';\n"
         "          end if;"),
        ("if(width_cntr = to_unsigned(0, width_cntr'length)) then",
         "if(width_zero = '1') then"),
        ("            width_cntr <= width_cntr -1;",
         "            width_cntr <= width_cntr -1;\n"
         "            if width_cntr = 1 then\n"
         "              width_zero <= '1';\n"
         "            end if;"),
    ]

# Sources ------------------------------------------------------------------------------------------

def phy8_source_overrides():
    """Portable replacements applied to copies of the pinned WR sources."""
    return {
        # GHDL treats unspecified std_logic states as a latch. The eight
        # binary selector values are already exhaustive.
        "wr-cores/ip_cores/general-cores/modules/wishbone/wb_uart/uart_async_tx.vhd": (
            "      when others => null;", "      when others => muxbit <= '0';"),
        # GW5 DPB does not support read-first. The diagnostic RAM explicitly
        # allows don't-care read-during-write data, so write-first is valid.
        "wr-cores/modules/wrc_core/wrc_diags_dpram.vhd": (
            'g_addr_conflict_resolution => "dont_care"',
            'g_addr_conflict_resolution => "write_first"'),
        # Give the falling-edge timestamp registers a local reset register.
        "wr-cores/modules/wr_endpoint/ep_timestamping_unit.vhd": [(
            "architecture syn of ep_timestamping_unit is",
            "architecture syn of ep_timestamping_unit is\n"
            "  signal rst_n_ref_f : std_logic := '0';"), (
            "  take_f : process(clk_ref_i)\n"
            "  begin\n"
            "    if falling_edge(clk_ref_i) then\n"
            "      if rst_n_ref_i = '0' then",
            "  p_reset_f : process(clk_ref_i)\n"
            "  begin\n"
            "    if falling_edge(clk_ref_i) then\n"
            "      rst_n_ref_f <= rst_n_ref_i;\n"
            "    end if;\n"
            "  end process;\n\n"
            "  take_f : process(clk_ref_i)\n"
            "  begin\n"
            "    if falling_edge(clk_ref_i) then\n"
            "      if rst_n_ref_f = '0' then")],
        # Factor the inversion bit out of the packet-filter ALU. This avoids
        # a deep eight-way operation mux after GHDL conversion on GW5.
        "wr-cores/modules/wr_endpoint/ep_packet_filter.vhd": (
            """    case op_t is
      when "000"  => r := a and b;
      when "100"  => r := a nand b;
      when "001"  => r := a or b;
      when "101"  => r := a nor b;
      when "010"  => r := a xor b;
      when "110"  => r := a xnor b;
      when "011"  => r := a;
      when "111"  => r := not a;
      when others => null;
    end case;
    return r;""",
            """    case op_t(1 downto 0) is
      when "00" => r := a and b;
      when "01" => r := a or b;
      when "10" => r := a xor b;
      when "11" => r := a;
      when others => r := 'X';
    end case;
    return r xor op_t(2);"""),
        # Track the pulse-width counter's zero state in a register.
        "wr-cores/modules/wr_pps_gen/xwr_pps_gen.vhd": pps_width_replacements(),
        # Let the synthesizer replicate the second synchronizer stage. WR
        # resynchronizes its network reset with gc_sync and distributes it to
        # a whole clock domain; on GW5A that single kept register cannot meet
        # 125 MHz across the receive PCS. Each replica is still driven by the
        # first stage, which keeps its attributes, so the two-register
        # synchronizer and its metastability margin are unchanged, and only
        # the fan-out registers are duplicated. keep_hierarchy would prevent
        # that duplication as well.
        "wr-cores/ip_cores/general-cores/modules/common/gc_sync.vhd": [(
            '  attribute keep of sync1          : signal is "true";',
            '  -- sync1 is replicable on GW5A; see the LiteX-WR-NIC overlay.'), (
            '  attribute keep_hierarchy         : string;\n'
            '  attribute keep_hierarchy of arch : architecture is "true";',
            '  -- keep_hierarchy would prevent that replication.')],
    }


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
    overrides = phy8_source_overrides()
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
                replacements = overrides[filename]
                if isinstance(replacements, tuple):
                    replacements = [replacements]
                for before, after in replacements:
                    _replace_once(copy, before, after)
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

    The hardware comma aligner does not report its slip, so the receive
    latency is not derived from a bitslide; it is a fixed offset of the link,
    measured with ``control.bitslide`` and the WR receive delay. The FIFO
    occupancies, the alignment-event counters and the PHY reset support that
    measurement across re-acquisitions.
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
        self.fifo_levels = CSRStatus(fields=[
            CSRField("rx", size=5, description="SerDes RX interface FIFO occupancy (words)."),
            CSRField("tx", size=5, description="SerDes TX interface FIFO occupancy (words)."),
        ])
        self.link_events = CSRStatus(fields=[
            CSRField("aligned", size=16, description="Comma alignment acquisitions since reset."),
            CSRField("lost",    size=16, description="Comma alignment losses since reset."),
        ])
        self.control = CSRStorage(fields=[
            CSRField("reset", size=1,
                description="Reset the SerDes and PCS, forcing CDR lock and comma alignment again."),
            CSRField("bitslide", size=4, offset=8,
                description="Receive bit-slip offset reported to WR, in 800 ps bit periods. "
                            "The hardware comma aligner does not report its slip; set a measured "
                            "value here, or leave it at zero and calibrate the receive delay."),
        ])

        # # #

        # SerDes clocks/resets.
        self.serdes = serdes = GW5SerDes(platform, lane=lane)
        self.tx_clk   = serdes.tx_clk
        self.rx_clk   = serdes.rx_clk
        self.pll_lock = serdes.pll_lock
        self.comb += [
            serdes.reset.eq(self.reset | self.control.fields.reset | ResetSignal("sys")),
            self.cd_wr_phy_tx.clk.eq(self.tx_clk),
            self.cd_wr_phy_rx.clk.eq(self.rx_clk),
            self.cd_wr_phy_ready.clk.eq(self.rx_clk),
            self.ready.eq(~self.cd_wr_phy_ready.rst),
            self.tx_error.eq(0),
            self.rx_bitslide.eq(self.control.fields.bitslide),
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
        # FIFO occupancies are steady once the link runs; alignment edges are
        # counted in the CSR domain from the synchronized flag.
        self.specials += MultiReg(Cat(serdes.rx_fifo_level, serdes.tx_fifo_level),
            Cat(self.fifo_levels.fields.rx, self.fifo_levels.fields.tx))
        aligned_d = Signal()
        self.sync += [
            aligned_d.eq(self.status.fields.aligned),
            If(self.status.fields.aligned & ~aligned_d,
                self.link_events.fields.aligned.eq(self.link_events.fields.aligned + 1)),
            If(~self.status.fields.aligned & aligned_d,
                self.link_events.fields.lost.eq(self.link_events.fields.lost + 1)),
        ]
