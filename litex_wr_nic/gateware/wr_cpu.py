#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from operator import xor
from functools import reduce

from migen import *

from litex.gen import LiteXModule

from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr import AutoCSR, CSRField, CSRStatus

from litex_wr_nic.wr_boot import (
    WR_BOOT_FLASH_OFFSET,
    WR_BOOT_MAGIC,
    WR_BOOT_MAX_PAYLOAD,
    WR_BOOT_VERSION,
)

# Constants ----------------------------------------------------------------------------------------

WR_CPU_MEMORY_ORIGIN = 0x4000_0000
WR_CPU_MEMORY_SIZE   = WR_BOOT_MAX_PAYLOAD

# WR CPU Word Addressing ---------------------------------------------------------------------------

def wr_cpu_word_bus(module, bus):
    """Normalize CPU masters before the SoC remaps them to high RAM addresses.

    Some LiteX versions report a byte-addressed Interface.address_width two
    bits narrower than its actual address signal. Cloning that interface in
    the SoC remapper then truncates bit 30 of WR_CPU_MEMORY_ORIGIN. Explicitly
    convert to the main bus's word addressing before any remapping/cloning.
    """
    if bus.addressing == "word":
        return bus
    if bus.data_width != 32 or bus.addressing != "byte":
        raise ValueError("WR CPU memory requires 32-bit Wishbone.")
    word_bus = wishbone.Interface(data_width=32, adr_width=len(bus.adr) - 2, addressing="word")
    module.comb += [
        bus.connect(word_bus, omit={"adr"}),
        word_bus.adr.eq(bus.adr[2:]),
    ]
    return word_bus

# WR CPU Memory Bridge -----------------------------------------------------------------------------

class WRCPUMemoryBridge(LiteXModule, AutoCSR):
    """Expose the flattened WR-core CPU-memory master as LiteX Wishbone."""

    def __init__(self, monitor_bus=None):
        self.bus = wishbone.Interface(data_width=32, address_width=32, addressing="byte")
        if monitor_bus is None:
            monitor_bus = self.bus

        self.cyc   = Signal()
        self.stb   = Signal()
        self.we    = Signal()
        self.adr   = Signal(32)
        self.sel   = Signal(4)
        self.dat_w = Signal(32)
        self.dat_r = Signal(32)
        self.ack   = Signal()
        self.err   = Signal()
        self.rty   = Signal()
        self.stall = Signal()

        self._status = CSRStatus(fields=[
            CSRField("error", description="A WR CPU memory transaction failed."),
            CSRField("busy",  description="A WR CPU memory transaction is active."),
        ])
        self._error_count        = CSRStatus(32, description="Number of WR CPU memory bus errors/timeouts.")
        self._last_error_address = CSRStatus(32, description="Address of the most recent failed transaction.")

        # # #

        error_sticky = Signal()
        error_count  = Signal(32)
        error_addr   = Signal(32)

        self.comb += [
            self.bus.cyc.eq(self.cyc),
            self.bus.stb.eq(self.stb),
            self.bus.we.eq(self.we),
            self.bus.adr.eq(self.adr),
            self.bus.sel.eq(self.sel),
            self.bus.dat_w.eq(self.dat_w),
            self.dat_r.eq(self.bus.dat_r),
            self.ack.eq(self.bus.ack),
            self.err.eq(self.bus.err),
            self.rty.eq(0),
            self.stall.eq(0),
            self._status.fields.error.eq(error_sticky),
            self._status.fields.busy.eq(monitor_bus.cyc),
            self._error_count.status.eq(error_count),
            self._last_error_address.status.eq(error_addr),
        ]
        # Monitor the system-clock side of the CDC when one is provided, so
        # error pulses and the multi-bit diagnostics remain in the CSR domain.
        self.sync += If(monitor_bus.err,
            error_sticky.eq(1),
            error_count.eq(error_count + 1),
            error_addr.eq(monitor_bus.adr),
        )

# CRC32 --------------------------------------------------------------------------------------------

def _crc32_byte(crc, data):
    # Build the reflected CRC transform as a linear XOR network. Keeping the
    # terms symbolic avoids the exponentially duplicated nested Mux tree that
    # a literal transcription of the software loop would create in Migen.
    value = [{n} for n in range(32)]
    for n in range(8):
        value[n] ^= {32 + n}
    for _ in range(8):
        lsb   = value[0]
        value = value[1:] + [set()]
        for n in range(32):
            if (0xedb8_8320 >> n) & 1:
                value[n] ^= lsb
    inputs = [crc[n] for n in range(32)] + [data[n] for n in range(8)]
    bits   = []
    for terms in value:
        bits.append(reduce(xor, (inputs[n] for n in terms), Constant(0, 1)))
    return Cat(*bits)

# WR CPU Flash Boot --------------------------------------------------------------------------------

class WRCPUFlashBoot(LiteXModule, AutoCSR):
    """Copy a packaged WR CPU image from 1-bit SPI flash to Wishbone memory."""

    ERROR_NONE       = 0
    ERROR_MAGIC      = 1
    ERROR_VERSION    = 2
    ERROR_LENGTH     = 3
    ERROR_CRC        = 4
    ERROR_WISHBONE   = 5
    ERROR_WB_TIMEOUT = 6

    def __init__(self, sys_clk_freq,
        flash_offset     = WR_BOOT_FLASH_OFFSET,
        max_payload      = WR_BOOT_MAX_PAYLOAD,
        spi_clk_freq     = 15e6,
        memory_wait      = 200e-6,
        wishbone_timeout = 1024,
    ):
        self.bus = wishbone.Interface(data_width=32, address_width=32, addressing="byte")

        self.clk   = Signal()
        self.cs_n  = Signal(reset=1)
        self.mosi  = Signal()
        self.miso  = Signal()
        self.owner = Signal(reset=1)
        self.ready = Signal()

        self._status = CSRStatus(fields=[
            CSRField("done",  description="The flash loader has completed."),
            CSRField("ready", description="The WR CPU image passed validation and is ready."),
            CSRField("owner", description="The loader currently owns the SPI flash."),
        ])
        self._error        = CSRStatus(8,  description="WR CPU flash-loader error code.")
        self._progress     = CSRStatus(32, description="Number of payload bytes copied.")
        self._length       = CSRStatus(32, description="Payload length from the boot header.")
        self._expected_crc = CSRStatus(32, description="CRC32 stored in the boot header.")
        self._actual_crc   = CSRStatus(32, description="CRC32 calculated by the loader.")

        # # #

        done         = Signal()
        error        = Signal(8)
        progress     = Signal(32)
        payload_len  = Signal(32)
        expected_crc = Signal(32)
        crc          = Signal(32, reset=0xffff_ffff)
        actual_crc   = Signal(32)
        remaining    = Signal(32)
        word_buffer  = Signal(32)
        word_address = Signal(32)
        byte_in_word = Signal(2)
        final_word   = Signal()
        wb_timeout   = Signal(max=wishbone_timeout + 1)

        self.comb += [
            self._status.fields.done.eq(done),
            self._status.fields.ready.eq(self.ready),
            self._status.fields.owner.eq(self.owner),
            self._error.status.eq(error),
            self._progress.status.eq(progress),
            self._length.status.eq(payload_len),
            self._expected_crc.status.eq(expected_crc),
            self._actual_crc.status.eq(actual_crc),
        ]

        # SPI mode-0 byte reader. It sends READ (0x03) and a 24-bit address,
        # then pauses between bytes whenever the image parser is busy.
        half_period = max(2, int(sys_clk_freq / (2 * spi_clk_freq)))
        spi_div     = Signal(max=half_period)
        spi_bits    = Signal(6)
        spi_tx      = Signal(32, reset=(0x03 << 24) | (flash_offset & 0x00ff_ffff))
        spi_rx      = Signal(8)
        byte_data   = Signal(8)
        byte_valid  = Signal()
        byte_accept = Signal()

        spi_fsm                 = FSM(reset_state="WAIT_MEMORY")
        self.submodules.spi_fsm = spi_fsm
        wait_cycles             = max(1, int(sys_clk_freq * memory_wait))
        wait_count              = Signal(max=wait_cycles + 1)

        spi_fsm.act("WAIT_MEMORY",
            self.cs_n.eq(1),
            If(wait_count == wait_cycles - 1,
                NextValue(spi_bits, 7),
                NextValue(spi_div, half_period - 1),
                NextState("WARMUP_LOW"),
            ).Else(
                NextValue(wait_count, wait_count + 1),
            )
        )
        # STARTUPE2 consumes the first three USRCCLKO cycles after EOS while
        # switching CCLK to user logic (UG470). Clock it with CS# inactive
        # before sending READ, otherwise those missing bits corrupt the command.
        spi_fsm.act("WARMUP_LOW",
            self.cs_n.eq(1),
            self.clk.eq(0),
            If(spi_div == 0,
                NextValue(spi_div, half_period - 1),
                NextState("WARMUP_HIGH"),
            ).Else(
                NextValue(spi_div, spi_div - 1),
            )
        )
        spi_fsm.act("WARMUP_HIGH",
            self.cs_n.eq(1),
            self.clk.eq(1),
            If(spi_div == 0,
                NextValue(spi_div, half_period - 1),
                If(spi_bits == 0,
                    NextValue(spi_bits, 31),
                    NextState("COMMAND_LOW"),
                ).Else(
                    NextValue(spi_bits, spi_bits - 1),
                    NextState("WARMUP_LOW"),
                )
            ).Else(
                NextValue(spi_div, spi_div - 1),
            )
        )
        spi_fsm.act("COMMAND_LOW",
            self.cs_n.eq(0),
            self.clk.eq(0),
            self.mosi.eq(spi_tx[31]),
            If(spi_div == 0,
                NextValue(spi_div, half_period - 1),
                NextState("COMMAND_HIGH"),
            ).Else(
                NextValue(spi_div, spi_div - 1),
            )
        )
        spi_fsm.act("COMMAND_HIGH",
            self.cs_n.eq(0),
            self.clk.eq(1),
            self.mosi.eq(spi_tx[31]),
            If(spi_div == 0,
                NextValue(spi_div, half_period - 1),
                NextValue(spi_tx, Cat(0, spi_tx[:-1])),
                If(spi_bits == 0,
                    NextValue(spi_bits, 7),
                    NextState("READ_LOW"),
                ).Else(
                    NextValue(spi_bits, spi_bits - 1),
                    NextState("COMMAND_LOW"),
                )
            ).Else(
                NextValue(spi_div, spi_div - 1),
            )
        )
        spi_fsm.act("READ_LOW",
            self.cs_n.eq(0),
            self.clk.eq(0),
            If(~self.owner,
                NextState("RELEASE"),
            ).Elif(byte_accept,
                NextValue(spi_div, half_period - 1),
                NextState("READ_HIGH"),
            )
        )
        spi_fsm.act("READ_HIGH",
            self.cs_n.eq(0),
            self.clk.eq(1),
            If(spi_div == 0,
                NextValue(spi_div, half_period - 1),
                NextValue(spi_rx, Cat(self.miso, spi_rx[:-1])),
                If(spi_bits == 0,
                    NextValue(byte_data, Cat(self.miso, spi_rx[:-1])),
                    NextValue(spi_bits, 7),
                    NextState("BYTE_READY"),
                ).Else(
                    NextValue(spi_bits, spi_bits - 1),
                    NextState("READ_LOW"),
                )
            ).Else(
                NextValue(spi_div, spi_div - 1),
            )
        )
        spi_fsm.act("BYTE_READY",
            self.cs_n.eq(0),
            self.clk.eq(0),
            byte_valid.eq(1),
            If(~self.owner,
                NextState("RELEASE"),
            ).Elif(byte_accept,
                NextState("READ_LOW"),
            )
        )
        spi_fsm.act("RELEASE",
            self.cs_n.eq(1),
            self.clk.eq(0),
        )

        # Boot image parser and Wishbone writer.
        header       = Array(Signal(8, name=f"header_{n}") for n in range(16))
        header_index = Signal(4)
        header_magic = Cat(*header[0:4])
        header_ver   = Cat(*header[4:8])
        header_len   = Cat(*header[8:12])
        header_crc   = Cat(*header[12:16])

        boot_fsm                 = FSM(reset_state="HEADER")
        self.submodules.boot_fsm = boot_fsm

        def fail(code):
            return [
                NextValue(error, code),
                NextValue(done, 1),
                NextValue(self.owner, 0),
                NextState("FAILED"),
            ]

        boot_fsm.act("HEADER",
            byte_accept.eq(1),
            If(byte_valid,
                NextValue(header[header_index], byte_data),
                If(header_index == 15,
                    NextState("CHECK_HEADER"),
                ).Else(
                    NextValue(header_index, header_index + 1),
                )
            )
        )
        boot_fsm.act("CHECK_HEADER",
            If(header_magic != int.from_bytes(WR_BOOT_MAGIC, "little"),
                *fail(self.ERROR_MAGIC),
            ).Elif(header_ver != WR_BOOT_VERSION,
                *fail(self.ERROR_VERSION),
            ).Elif((header_len == 0) | (header_len[:2] != 0) | (header_len > max_payload),
                *fail(self.ERROR_LENGTH),
            ).Else(
                NextValue(payload_len, header_len),
                NextValue(remaining, header_len),
                NextValue(expected_crc, header_crc),
                NextValue(crc, 0xffff_ffff),
                NextValue(progress, 0),
                NextState("PAYLOAD"),
            )
        )
        boot_fsm.act("PAYLOAD",
            byte_accept.eq(1),
            If(byte_valid,
                NextValue(crc, _crc32_byte(crc, byte_data)),
                NextValue(progress, progress + 1),
                NextValue(remaining, remaining - 1),
                If(byte_in_word == 0,
                    NextValue(word_buffer, byte_data),
                ).Elif(byte_in_word == 1,
                    NextValue(word_buffer, word_buffer | (byte_data << 8)),
                ).Elif(byte_in_word == 2,
                    NextValue(word_buffer, word_buffer | (byte_data << 16)),
                ).Else(
                    NextValue(self.bus.dat_w, Cat(word_buffer[:24], byte_data)),
                    NextValue(self.bus.adr, word_address),
                    NextValue(self.bus.sel, 0xf),
                    NextValue(self.bus.we, 1),
                    NextValue(self.bus.cyc, 1),
                    NextValue(self.bus.stb, 1),
                    NextValue(final_word, remaining == 1),
                    NextValue(byte_in_word, 0),
                    NextValue(wb_timeout, 0),
                    NextState("WRITE"),
                ),
                If(byte_in_word != 3,
                    NextValue(byte_in_word, byte_in_word + 1),
                )
            )
        )
        boot_fsm.act("WRITE",
            # This is a classic Wishbone master: the cache can acknowledge
            # several cycles later, so keep STB asserted until completion.
            If(self.bus.ack,
                NextValue(self.bus.cyc, 0),
                NextValue(self.bus.stb, 0),
                NextValue(word_address, word_address + 4),
                If(final_word,
                    NextState("CHECK_CRC"),
                ).Else(
                    NextState("PAYLOAD"),
                )
            ).Elif(self.bus.err,
                NextValue(self.bus.cyc, 0),
                NextValue(self.bus.stb, 0),
                *fail(self.ERROR_WISHBONE),
            ).Elif(wb_timeout == wishbone_timeout - 1,
                NextValue(self.bus.cyc, 0),
                NextValue(self.bus.stb, 0),
                *fail(self.ERROR_WB_TIMEOUT),
            ).Else(
                NextValue(wb_timeout, wb_timeout + 1),
            )
        )
        boot_fsm.act("CHECK_CRC",
            NextValue(actual_crc, crc ^ 0xffff_ffff),
            If((crc ^ 0xffff_ffff) != expected_crc,
                *fail(self.ERROR_CRC),
            ).Else(
                NextValue(self.ready, 1),
                NextValue(self.owner, 0),
                NextValue(done, 1),
                NextState("SUCCESS"),
            )
        )
        # Empty FSM actions are omitted by Migen and fall through to HEADER,
        # which is the default case. Keep both terminal states explicit.
        boot_fsm.act("SUCCESS", NextState("SUCCESS"))
        boot_fsm.act("FAILED", NextState("FAILED"))
