#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Packet-buffered WR fabric bridge to a gigabit RGMII interface."""

from types import SimpleNamespace

from migen import *
from migen.genlib.cdc import MultiReg
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import LiteXModule

from litex.build.io import DDROutput
from litex.soc.cores.clock import S7PLL
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import CSRStatus

from liteeth.common import eth_phy_description
from liteeth.mac.core import LiteEthMACCore

# WR Packet Buffer --------------------------------------------------------------------------------

class WRPacketBuffer(LiteXModule):
    """Accept an unthrottled input and publish only complete, valid frames.

    Each slot is held until its final output byte is accepted. A full buffer,
    a receive error, or a frame larger than a slot discards the entire frame.
    The synchronous RAM output sustains one byte per clock within a frame.
    """
    def __init__(self, depth=2048, slots=2, with_first=False):
        if depth < 2 or depth & (depth - 1) or slots < 2 or slots & (slots - 1):
            raise ValueError("Packet depth and slot count must be powers of two, at least two")
        self.sink   = sink   = stream.Endpoint(eth_phy_description(8))
        self.source = source = stream.Endpoint(eth_phy_description(8))
        # Abort only the input capture; completed frames remain deliverable.
        self.clear = Signal()
        self.packets = CSRStatus(32, description="Complete frames accepted by this buffer.")
        self.dropped = CSRStatus(32, description="Frames discarded on error, oversize or buffer exhaustion.")

        # # #

        # Storage.
        offset_bits = log2_int(depth)
        slot_bits   = log2_int(slots)
        self.descriptors = descriptors = stream.SyncFIFO([
            ("slot",   slot_bits),
            ("length", offset_bits + 1),
        ], slots, buffered=True)
        memory = Memory(8, depth * slots)
        write  = memory.get_port(write_capable=True)
        read   = memory.get_port(has_re=True)
        self.specials += memory, write, read

        # Capture.
        write_slot = Signal(slot_bits)
        occupied   = Signal(max=slots + 1)
        length     = Signal(max=depth + 1)
        discard    = Signal()
        bad        = Signal()
        self.comb += [
            sink.ready.eq(1),
            bad.eq(discard | sink.error | (length == depth) |
                ((length == 0) & ~sink.first & with_first) |
                ((length == 0) & ((occupied == slots) | ~descriptors.sink.ready))),
            write.adr.eq(Cat(length[:offset_bits], write_slot)),
            write.dat_w.eq(sink.data),
            write.we.eq(sink.valid & ~bad & ~self.clear),
            descriptors.sink.valid.eq(sink.valid & sink.last & ~bad & ~self.clear),
            descriptors.sink.slot.eq(write_slot),
            descriptors.sink.length.eq(length + 1),
        ]
        self.sync += If(self.clear,
            If(length != 0, self.dropped.status.eq(self.dropped.status + 1)),
            length.eq(0),
            discard.eq(0),
        ).Elif(sink.valid,
            If(sink.last,
                length.eq(0),
                discard.eq(0),
                If(bad,
                    self.dropped.status.eq(self.dropped.status + 1),
                ).Else(
                    write_slot.eq(write_slot + 1),
                    self.packets.status.eq(self.packets.status + 1),
                ),
            ).Else(
                discard.eq(bad),
                If(length != depth, length.eq(length + 1)),
            ),
        )

        # Readout.
        read_offset  = Signal(offset_bits)
        output_valid = Signal()
        output_last  = Signal()
        output_first = Signal()
        advance      = Signal()
        issue        = Signal()
        self.comb += [
            advance.eq(~output_valid | source.ready),
            issue.eq(advance & descriptors.source.valid & ~(output_valid & output_last)),
            read.adr.eq(Cat(read_offset, descriptors.source.slot)),
            read.re.eq(issue),
            source.valid.eq(output_valid),
            source.first.eq(output_first),
            source.last.eq(output_last),
            source.last_be.eq(output_last),
            source.data.eq(read.dat_r),
            descriptors.source.ready.eq(source.valid & source.ready & source.last),
        ]
        self.sync += [
            Case(Cat(descriptors.sink.valid & descriptors.sink.ready,
                     descriptors.source.valid & descriptors.source.ready), {
                1: occupied.eq(occupied + 1),
                2: occupied.eq(occupied - 1),
            }),
            If(advance, output_valid.eq(issue)),
            If(issue,
                output_first.eq(read_offset == 0),
                output_last.eq(read_offset == descriptors.source.length - 1),
                If(read_offset == descriptors.source.length - 1,
                    read_offset.eq(0),
                ).Else(
                    read_offset.eq(read_offset + 1),
                ),
            ),
        ]

# WR RGMII PHY ------------------------------------------------------------------------------------

class WRRGMIIPhy(LiteXModule):
    """Fixed 1 Gb/s DDR interface, with locally generated transmit clock.

    In MAC role the board drives TX pins. In PHY role it drives RX pins to
    connect to another MAC. Delay values are FPGA-added clock shifts; a peer
    that already delays its clock requires a corresponding zero FPGA delay.
    """
    dw          = 8
    tx_clk_freq = 125e6
    rx_clk_freq = 125e6

    def __init__(self, platform, pads, role="mac", tx_delay=2e-9, rx_delay=2e-9,
        rx_phase_adjust=0):
        if role not in ("mac", "phy"):
            raise ValueError("RGMII role must be mac or phy")
        if not (0 <= tx_delay <= 2e-9 and 0 <= rx_delay <= 2e-9):
            raise ValueError("FPGA RGMII clock delays must be in [0, 2] ns")
        self.sink   = sink   = stream.Endpoint(eth_phy_description(8))
        self.source = source = stream.Endpoint(eth_phy_description(8))

        # # #

        # Pad Direction.
        if role == "phy":
            pads = SimpleNamespace(
                tx_clk  = pads.rx_clk,
                tx_en   = pads.rx_dv,
                tx_data = pads.rx_data,
                rx_clk  = pads.tx_clk,
                rx_dv   = pads.tx_en,
                rx_data = pads.tx_data,
            )

        # TX Clock.
        self.cd_eth_tx         = ClockDomain()
        self.cd_eth_tx_delayed = ClockDomain(reset_less=True)
        self.tx_pll = tx_pll = S7PLL(speedgrade=-2)
        self.comb += tx_pll.reset.eq(ResetSignal("sys"))
        tx_pll.register_clkin(ClockSignal("sys"), 125e6)
        tx_pll.create_clkout(self.cd_eth_tx, 125e6, margin=0, with_reset=False)
        tx_pll.create_clkout(self.cd_eth_tx_delayed, 125e6, phase=tx_delay * 125e6 * 360, margin=0)
        self.specials += Instance("ODDR", name="rgmii_tx_clock",
            p_DDR_CLK_EDGE = "SAME_EDGE",
            i_C  = ClockSignal("eth_tx_delayed"),
            i_CE = 1,
            i_S  = 0,
            i_R  = 0,
            i_D1 = 1,
            i_D2 = 0,
            o_Q  = pads.tx_clk,
        )

        # RX Clock.
        # A separate PLL centers the sampling clock within the received DDR
        # data eye. Retry from the free-running system clock if the peer is
        # absent or its clock stops: PLLE2 requires reset after loss of lock.
        self.cd_eth_rx = ClockDomain()
        rx_locked     = Signal()
        rx_locked_sys = Signal()
        rx_reset      = Signal()
        retry_count   = Signal(16)
        self.specials += MultiReg(rx_locked, rx_locked_sys)
        self.sync += If(rx_locked_sys,
            retry_count.eq(0),
        ).Else(
            retry_count.eq(retry_count + 1),
        )
        # A 256 ns reset every 524 us leaves time for PLL acquisition. Keep
        # retrying until a clock is present and the PLL can acquire lock.
        self.comb += rx_reset.eq(ResetSignal("sys") | (~rx_locked_sys & (retry_count < 32)))
        rx_feedback     = Signal()
        rx_feedback_buf = Signal()
        rx_clock        = Signal()
        # Buffer the feedback to compensate the receive clock tree. S7PLL's
        # direct feedback leaves the BUFG insertion delay uncompensated,
        # moving the IDDR sampling edge outside the 4 ns data eye. BUF_IN
        # avoids ZHOLD's additional input deskew; the board can adjust phase
        # for its clock/data input-path skew, which remains covered by STA.
        self.specials += [
            Instance("PLLE2_ADV", name="rgmii_rx_pll",
                p_BANDWIDTH      = "OPTIMIZED",
                p_COMPENSATION   = "BUF_IN",
                p_CLKIN1_PERIOD  = 8.0,
                p_REF_JITTER1    = 0.01,
                p_CLKFBOUT_MULT  = 8,
                p_DIVCLK_DIVIDE  = 1,
                p_CLKOUT0_DIVIDE = 8,
                p_CLKOUT0_PHASE  = ((rx_delay + rx_phase_adjust) * 125e6 * 360) % 360,
                i_CLKIN1   = pads.rx_clk,
                i_CLKIN2   = 0,
                i_CLKINSEL = 1,
                i_RST      = rx_reset,
                i_PWRDWN   = 0,
                i_DCLK     = 0,
                i_DEN      = 0,
                i_DWE      = 0,
                i_DADDR    = 0,
                i_DI       = 0,
                i_CLKFBIN  = rx_feedback_buf,
                o_CLKFBOUT = rx_feedback,
                o_CLKOUT0  = rx_clock,
                o_LOCKED   = rx_locked,
            ),
            Instance("BUFG", i_I=rx_feedback, o_O=rx_feedback_buf),
            Instance("BUFG", i_I=rx_clock, o_O=self.cd_eth_rx.clk),
        ]

        # Datapath Reset.
        self.reset = Signal()
        self.comb += self.reset.eq(ResetSignal("sys") | ~tx_pll.locked | ~rx_locked)
        self.clock_ready = CSRStatus(description="Both RGMII PLLs are locked; this is not Ethernet link status.")
        self.specials += MultiReg(~self.reset, self.clock_ready.status)
        self.specials += [
            AsyncResetSynchronizer(self.cd_eth_tx, self.reset),
            AsyncResetSynchronizer(self.cd_eth_rx, self.reset),
        ]

        # TX Datapath.
        self.comb += sink.ready.eq(1)
        tx_valid = Signal()
        self.comb += tx_valid.eq(sink.valid & ~ResetSignal("eth_tx"))
        self.specials += [
            DDROutput(sink.data[:4], sink.data[4:], pads.tx_data, clk=ClockSignal("eth_tx")),
            DDROutput(tx_valid, tx_valid & ~sink.error, pads.tx_en, clk=ClockSignal("eth_tx")),
        ]

        # RX Datapath.
        data_rise, data_fall = Signal(4), Signal(4)
        ctl_rise,  ctl_fall  = Signal(),  Signal()
        # SAME_EDGE_PIPELINED pairs the two nibbles of the same byte. The
        # generic SAME_EDGE DDR input presents adjacent half-cycles instead.
        for pad, rise, fall in [(pads.rx_data[n], data_rise[n], data_fall[n])
                               for n in range(4)] + [(pads.rx_dv, ctl_rise, ctl_fall)]:
            self.specials += Instance("IDDR",
                p_DDR_CLK_EDGE = "SAME_EDGE_PIPELINED",
                i_C  = ClockSignal("eth_rx"),
                i_CE = 1,
                i_S  = 0,
                i_R  = 0,
                i_D  = pad,
                o_Q1 = rise,
                o_Q2 = fall,
            )
        frame_error = Signal()
        rx_valid    = Signal()
        self.sync.eth_rx += [
            rx_valid.eq(ctl_rise),
            source.data.eq(Cat(data_rise, data_fall)),
            source.error.eq(frame_error | (ctl_rise ^ ctl_fall)),
            If(ctl_rise,
                frame_error.eq(frame_error | (ctl_rise ^ ctl_fall)),
            ).Else(
                frame_error.eq(0),
            ),
        ]
        self.comb += [
            source.valid.eq(rx_valid & ~ResetSignal("eth_rx")),
            source.last.eq(source.valid & ~ctl_rise),
            source.last_be.eq(source.last),
        ]

        # Timing Constraints.
        platform.add_false_path_constraints(self.cd_eth_rx.clk, self.cd_eth_tx.clk)

        # Budget a 2 ns clock/data offset in each direction, shared between
        # FPGA and peer. RGMII source skew is +/-0.5 ns; receiver setup/hold
        # are 1 ns. Represent the peer's delay in the clock waveforms so
        # setup/hold use the correct DDR edges even for zero FPGA delay.
        peer_rx_delay = 2 - rx_delay * 1e9
        peer_tx_delay = 2 - tx_delay * 1e9
        platform.add_platform_command("create_clock -name rgmii_rx_input -period 8.0 "
            + f"-waveform [list {peer_rx_delay:.3f} {peer_rx_delay + 4:.3f}] "
            + "[get_ports {clk}]", clk=pads.rx_clk)
        platform.add_platform_command("create_clock -name rgmii_rx_launch -period 8.0")
        commands = platform.toolchain.pre_placement_commands
        commands.add("create_generated_clock -name rgmii_tx_capture "
            "-source [get_pins rgmii_tx_clock/C] -edges [list 1 2 3] "
            + f"-edge_shift [list {peer_tx_delay:.3f} {peer_tx_delay:.3f} {peer_tx_delay:.3f}] "
            + "[get_ports {tx_clk}]",
            tx_clk=pads.tx_clk)
        commands.add("set_property SLEW FAST [get_ports [list {data}* {ctl} {clk}]]",
            data=pads.tx_data, ctl=pads.tx_en, clk=pads.tx_clk)
        for edge in ("", " -clock_fall -add_delay"):
            for bound, value in (("min", -0.5), ("max", 0.5)):
                commands.add("set_input_delay -clock rgmii_rx_launch "
                    + f"-{bound} {value:.3f}{edge} " + "[get_ports [list {data}* {ctl}]]",
                    data=pads.rx_data, ctl=pads.rx_dv)
            for bound, value in (("min", -1), ("max", 1)):
                commands.add("set_output_delay -clock rgmii_tx_capture "
                    + f"-{bound} {value:.3f}{edge} " + "[get_ports [list {data}* {ctl}]]",
                    data=pads.tx_data, ctl=pads.tx_en)

# WR RGMII Bridge ---------------------------------------------------------------------------------

class WRRGMIIBridge(LiteXModule):
    def __init__(self, phy, wr_source, wr_sink, wr_error=0, depth=2048, slots=2):
        # Reset both ends of the MAC crossings on peer clock loss. Preserve
        # completed RX frames: WR may already be consuming one of them.
        self.cd_rgmii_sys = ClockDomain()
        self.comb += self.cd_rgmii_sys.clk.eq(ClockSignal("sys"))
        self.specials += AsyncResetSynchronizer(self.cd_rgmii_sys,
            getattr(phy, "reset", ResetSignal("sys")),
        )

        # Packet Buffers.
        self.tx_buffer = ClockDomainsRenamer("rgmii_sys")(WRPacketBuffer(depth, slots, with_first=True))
        self.rx_buffer = WRPacketBuffer(depth, slots)
        self.specials += MultiReg(ResetSignal("rgmii_sys"), self.rx_buffer.clear)
        # MAC.
        self.mac = ClockDomainsRenamer({"sys": "rgmii_sys"})(LiteEthMACCore(phy,
            dw           = 8,
            eth_mtu      = depth,
            tx_cdc_depth = 64,
            rx_cdc_depth = 64,
        ))

        # WR Fabric -> TX Buffer -> MAC -> RX Buffer -> WR Fabric.
        self.comb += [
            wr_source.connect(self.tx_buffer.sink),
            self.tx_buffer.sink.error.eq(wr_error),
            self.tx_buffer.source.connect(self.mac.sink),
            self.mac.source.connect(self.rx_buffer.sink),
            self.rx_buffer.source.connect(wr_sink, omit={"error", "last_be"}),
        ]
