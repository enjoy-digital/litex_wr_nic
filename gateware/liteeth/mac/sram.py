#
# This file is part of LiteEth.
#
# Copyright (c) 2015-2021 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2015-2018 Sebastien Bourdeauducq <sb@m-labs.hk>
# Copyright (c) 2021 Leon Schuermann <leon@is.currently.online>
# Copyright (c) 2017 whitequark <whitequark@whitequark.org>
# SPDX-License-Identifier: BSD-2-Clause

import math

from liteeth.common import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect.csr_eventmanager import *

# MAC SRAM Writer ----------------------------------------------------------------------------------

class LiteEthMACSRAMWriter(Module, AutoCSR):
    def __init__(self, dw, depth, nslots=2, endianness="big", timestamp=None, with_eth_pcie=True):
        # Endpoint / Signals.
        self.sink      = sink = stream.Endpoint(eth_phy_description(dw))
        self.crc_error = Signal()

        # Parameters Check / Compute.
        assert dw in [8, 16, 32, 64, 128]
        slotbits   = max(int(math.log2(nslots)), 1)
        lengthbits = bits_for(depth * dw//8)

        # CSRs.
        self._slot   = CSRStatus(slotbits)
        self._length = CSRStatus(lengthbits)
        self._errors = CSRStatus(32)
        self._discard = CSRStatus(32,reset=0)
        if with_eth_pcie:
            self._enable          = CSRStorage(reset=0)
            self.start_transfer   = Signal(reset=0)
            self.transfer_ready   = Signal(reset=0)
            self.test1            = CSRStatus(32,reset=0)
            self.test2            = CSRStatus(32,reset=0)
            self.test3            = CSRStatus(32,reset=0)
            self._pending_slots   = CSRStatus(nslots,reset=0)
            self._clear_pending   = CSRStorage(nslots,reset=0)
            self._pending_length  = CSRStatus(32*nslots,reset=0)
            self._pcie_host_addrs = CSRStorage(32*nslots,reset=0)

        # Optional Timestamp of the incoming packets and expose value to software.
        if timestamp is not None:
            timestampbits   = len(timestamp)
            self._timestamp = CSRStatus(timestampbits)

        # Event Manager.
        self.submodules.ev = EventManager()
        self.ev.available  = EventSourceLevel()
        self.ev.finalize()

        # # #

        enable = Signal()
        if with_eth_pcie:
            self.pcie_irq       = Signal()
            stat_fifo_valid_tmp = Signal()

            self.pcie_slot      = Signal(32,reset=0)
            self.pcie_host_addr = Signal(32,reset=0)
            self.comb += enable.eq(self._enable.storage)
        else:
            self.comb += enable.eq(1)
        write   = Signal()
        errors  = self._errors.status

        slot       = Signal(slotbits)
        length     = Signal(lengthbits)
        length_inc = Signal(5)

        # Sink is already ready: packets are dropped when no slot is available.
        sink.ready.reset = 1

        # Decode Length increment from from last_be.
        self.comb += Case(sink.last_be, {
            0b00000001 : length_inc.eq(1),
            0b00000010 : length_inc.eq(2),
            0b00000100 : length_inc.eq(3),
            0b00001000 : length_inc.eq(4),
            0b00010000 : length_inc.eq(5),
            0b00100000 : length_inc.eq(6),
            0b01000000 : length_inc.eq(7),
            0b010000000: length_inc.eq(8),
            0b0100000000: length_inc.eq(9),
            0b01000000000: length_inc.eq(10),
            0b010000000000: length_inc.eq(11),
            0b0100000000000: length_inc.eq(12),
            0b01000000000000: length_inc.eq(13),
            0b010000000000000: length_inc.eq(14),
            0b0100000000000000: length_inc.eq(15),
            "default"  : length_inc.eq(dw//8)
        })

        # Status FIFO.
        stat_fifo_layout = [("slot", slotbits), ("length", lengthbits)]
        if timestamp is not None:
            stat_fifo_layout += [("timestamp", timestampbits)]
        self.submodules.stat_fifo = stat_fifo = stream.SyncFIFO(stat_fifo_layout, nslots)

        # FSM.
        self.submodules.fsm = fsm = FSM(reset_state="WRITE")
        fsm.act("WRITE",
            If(sink.valid & enable,
                If(stat_fifo.sink.ready,
                    write.eq(1),
                    NextValue(length, length + length_inc),
                    If(length >= eth_mtu,
                         NextState("DISCARD-REMAINING")
                    ),
                    If(sink.last,
                        If((sink.error & sink.last_be) != 0,
                            NextState("DISCARD")
                        ).Else(
                            NextState("TERMINATE")
                        )
                    )
                ).Else(
                    NextValue(errors, errors + 1),
                    NextState("DISCARD-ALL")
                )
            )
        )
        fsm.act("DISCARD-REMAINING",
            If(sink.valid & sink.last,
                If((sink.error & sink.last_be) != 0,
                    NextValue(self._discard.status,self._discard.status+1),
                    NextState("DISCARD")
                ).Else(
                    NextState("TERMINATE")
                )
            )
        )
        fsm.act("DISCARD-ALL",
            If(sink.valid & sink.last,
                If((sink.last_be) != 0,
                    NextState("DISCARD")
                ).Else(
                    NextValue(length, 0),
                    NextState("WRITE")
                )
            )
        )
        fsm.act("DISCARD",
            NextValue(length, 0),
            NextState("WRITE")
        )
        fsm.act("TERMINATE",
            stat_fifo.sink.valid.eq(1),
            stat_fifo.sink.slot.eq(slot),
            stat_fifo.sink.length.eq(length),
            NextValue(length, 0),
            NextValue(slot, slot + 1),
            NextState("WRITE")
        )

        if not with_eth_pcie:
            self.comb += [
                stat_fifo.source.ready.eq(self.ev.available.clear),
                self.ev.available.trigger.eq(stat_fifo.source.valid),
            ]
        else:
            self.comb += self.test3.status.eq(stat_fifo.level)

        self.comb += [
            self._slot.status.eq(stat_fifo.source.slot),
            self._length.status.eq(stat_fifo.source.length),
        ]
        if timestamp is not None:
            # Latch Timestamp on start of packet.
            self.sync += If(length == 0, stat_fifo.sink.timestamp.eq(timestamp))
            self.comb += self._timestamp.status.eq(stat_fifo.source.timestamp)

        if with_eth_pcie:
            self.sync += [
                If(stat_fifo.source.valid,self.test1.status.eq(self.test1.status+1)),
                If(self.pcie_irq, self.test2.status.eq(self.test2.status + 1))
            ]
            self.submodules.irq_fsm = irq_fsm = FSM(reset_state="IDLE")

            self.comb += self.pcie_slot.eq(0xffffffff),
            for i in reversed(range(nslots)): # Priority given to lower indexes.
                self.comb += If(self._pending_slots.status[i] == 0, self.pcie_slot.eq(i))

            clear_pending = Signal(32,reset=0)
            new_pending_slots = Signal(32,reset=0)
            pending_length = Array(Signal(32,reset=0) for i in range(nslots))
            pcie_host_addrs = Array(Signal(32,reset=0) for i in range(nslots))

            for i in range(nslots):
                self.comb += [
                    self._pending_length.status[i*32:(i+1)*32].eq(pending_length[nslots-i-1]),
                ]

            for i in range(nslots):
                self.comb += [
                    pcie_host_addrs[nslots-i-1].eq(self._pcie_host_addrs.storage[i*32:(i+1)*32]),
                ]

            self.comb += [If(self._clear_pending.re, clear_pending.eq(self._clear_pending.storage)),
                          If(self.start_transfer,
                             new_pending_slots.eq(1 << self.pcie_slot))]

            self.sync += self._pending_slots.status.eq((self._pending_slots.status & ~clear_pending) | new_pending_slots)

            irq_fsm.act("IDLE",
                    If(stat_fifo.source.valid & (self.pcie_slot != 0xffffffff),
                       NextValue(pending_length[self.pcie_slot],stat_fifo.source.length),
                       NextValue(self.pcie_host_addr,pcie_host_addrs[self.pcie_slot]),
                       NextState("TRANSFER")),
            )
            irq_fsm.act("TRANSFER",
                    self.start_transfer.eq(1), NextState("WAIT_TRANSFER"),
            )
            irq_fsm.act("WAIT_TRANSFER",
                    If(self.transfer_ready,
                       self.pcie_irq.eq(1),
                       stat_fifo.source.ready.eq(1),
                       NextState("IDLE")),
            )

        # Memory.
        wr_slot = slot
        wr_addr = length[int(math.log2(dw//8)):]
        wr_data = Signal(len(sink.data))

        # Create a Memory per Slot.
        mems  = [None] * nslots
        ports = [None] * nslots
        for n in range(nslots):
            mems[n]  = Memory(dw, depth)
            ports[n] = mems[n].get_port(write_capable=True)
            self.specials += ports[n]
        self.mems = mems

        # Endianness Handling.
        self.comb += wr_data.eq({"big": reverse_bytes(sink.data), "little": sink.data}[endianness])

        # Connect Memory ports.
        cases = {}
        for n, port in enumerate(ports):
            cases[n] = [
                ports[n].adr.eq(wr_addr),
                ports[n].dat_w.eq(wr_data),
                If(sink.valid & write,
                    ports[n].we.eq(2**len(ports[n].we) - 1)
                )
            ]
        self.comb += Case(wr_slot, cases)

# MAC SRAM Reader ----------------------------------------------------------------------------------

class LiteEthMACSRAMReader(Module, AutoCSR):
    def __init__(self, dw, depth, nslots=2, endianness="big", timestamp=None, with_eth_pcie=True):
        # Endpoint / Signals.
        self.source = source = stream.Endpoint(eth_phy_description(dw))

        # Parameters Check / Compute.
        assert dw in [8, 16, 32, 64, 128]
        slotbits   = max(int(math.log2(nslots)), 1)
        lengthbits = bits_for(depth * dw//8)

        # CSRs.
        self._start  = CSR()
        self._ready  = CSRStatus()
        self._level  = CSRStatus(int(math.log2(nslots)) + 1)
        self._slot   = CSRStorage(slotbits,   reset_less=True)
        self._length = CSRStorage(lengthbits, reset_less=True)
        if with_eth_pcie:
            self.start_transfer   = Signal(reset=0)
            self.transfer_ready   = Signal(reset=0)
            self._pcie_host_addrs = CSRStorage(32*nslots,reset=0)
            self._pending_slots = CSRStatus(nslots,reset=0)
            self._clear_pending = CSRStorage(nslots,reset=0)

        # Optional Timestamp of the outgoing packets and expose value to software.
        if timestamp is not None:
            timestampbits        = len(timestamp)
            self._timestamp_slot = CSRStatus(slotbits)
            self._timestamp      = CSRStatus(timestampbits)

        # Event Manager.
        self.submodules.ev = EventManager()
        self.ev.done       = EventSourcePulse() if timestamp is None else EventSourceLevel()
        self.ev.finalize()

        # # #

        read          = Signal()
        length        = Signal(lengthbits)
        event_trigger = Signal()
        if with_eth_pcie:
            slot_size = CSRConstant(2 ** bits_for(eth_mtu))
            self.pcie_irq = Signal()
            self.pcie_host_addr = Signal(32,reset=0)

            clear_pending = Signal(32,reset=0)
            new_pending_slots = Signal(32,reset=0)
            pcie_host_addrs = Array(Signal(32,reset=0) for i in range(nslots))

            for i in range(nslots):
                self.comb += [
                    pcie_host_addrs[nslots-i-1].eq(self._pcie_host_addrs.storage[i*32:(i+1)*32]),
                ]
            self.comb += self.pcie_irq.eq(event_trigger)
        else:
            self.comb += self.ev.done.trigger.eq(event_trigger)

        # Command FIFO.
        self.cmd_fifo = cmd_fifo = stream.SyncFIFO([("slot", slotbits), ("length", lengthbits)], nslots)
        self.submodules += cmd_fifo
        self.comb += [
            cmd_fifo.sink.valid.eq(self._start.re),
            cmd_fifo.sink.slot.eq(self._slot.storage),
            cmd_fifo.sink.length.eq(self._length.storage),
            self._ready.status.eq(cmd_fifo.sink.ready),
            self._level.status.eq(cmd_fifo.level)
        ]

        if with_eth_pcie:
            self.comb += [
                self.pcie_host_addr.eq(pcie_host_addrs[cmd_fifo.source.slot]),
                If(self._clear_pending.re,
                    clear_pending.eq(self._clear_pending.storage)
                ),
                If(cmd_fifo.source.ready,
                    new_pending_slots.eq(1 << cmd_fifo.source.slot)
                )
            ]

            self.sync += self._pending_slots.status.eq((self._pending_slots.status & ~clear_pending) | new_pending_slots)

        # Status FIFO (Only added when Timestamping).
        if timestamp is not None:
            stat_fifo_layout = [("slot", slotbits), ("timestamp", timestampbits)]
            stat_fifo = stream.SyncFIFO(stat_fifo_layout, nslots)
            self.submodules += stat_fifo
            self.comb += stat_fifo.source.ready.eq(self.ev.done.clear)
            self.comb += self._timestamp_slot.status.eq(stat_fifo.source.slot)
            self.comb += self._timestamp.status.eq(stat_fifo.source.timestamp)

        # Encode Length to last_be.
        length_lsb = cmd_fifo.source.length[:int(math.log2(dw/8))] if (dw != 8) else 0
        self.comb += If(source.last,
            Case(length_lsb, {
                1         : source.last_be.eq(0b00000001),
                2         : source.last_be.eq(0b00000010),
                3         : source.last_be.eq(0b00000100),
                4         : source.last_be.eq(0b00001000),
                5         : source.last_be.eq(0b00010000),
                6         : source.last_be.eq(0b00100000),
                7         : source.last_be.eq(0b01000000),
                8         : source.last_be.eq(0b010000000),
                9         : source.last_be.eq(0b0100000000),
                10        : source.last_be.eq(0b01000000000),
                11        : source.last_be.eq(0b010000000000),
                12        : source.last_be.eq(0b0100000000000),
                13        : source.last_be.eq(0b01000000000000),
                14        : source.last_be.eq(0b010000000000000),
                15        : source.last_be.eq(0b0100000000000000),
                "default" : source.last_be.eq(2**(dw//8 - 1)),
            })
        )

        # FSM.
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        if with_eth_pcie:
            fsm.act("IDLE",
                If(cmd_fifo.source.valid,
                   self.start_transfer.eq(1),
                   NextState("WAIT_PCIE"),
                )
            )
            fsm.act("WAIT_PCIE",
                If(self.transfer_ready,
                    read.eq(1),
                    NextValue(length, dw//8),
                    NextState("READ")
                )
            )
        else:
            fsm.act("IDLE",
                If(cmd_fifo.source.valid,
                    read.eq(1),
                    NextValue(length, dw//8),
                    NextState("READ")
                )
            )
        fsm.act("READ",
            source.valid.eq(1),
            source.last.eq(length >= cmd_fifo.source.length),
            If(source.ready,
                read.eq(1),
                NextValue(length, length + dw//8),
                If(source.last,
                    NextState("TERMINATE")
                )
            )
        )
        fsm.act("TERMINATE",
            NextValue(length, 0),
            event_trigger.eq(1),
            cmd_fifo.source.ready.eq(1),
            NextState("IDLE")
        )

        if timestamp is not None:
            # Latch Timestamp on start of outgoing packet.
            self.sync += If(length == 0, stat_fifo.sink.timestamp.eq(timestamp))
            self.comb += stat_fifo.sink.valid.eq(fsm.ongoing("END"))
            self.comb += stat_fifo.sink.slot.eq(cmd_fifo.source.slot)
            # Trigger event when Status FIFO has contents (Override FSM assignment).
            self.comb += self.ev.done.trigger.eq(stat_fifo.source.valid)

        # Memory.
        rd_slot = cmd_fifo.source.slot
        rd_addr = Signal(lengthbits)
        rd_data = Signal(len(source.data))

        # Create a Memory per Slot.
        mems    = [None]*nslots
        ports   = [None]*nslots
        for n in range(nslots):
            mems[n]  = Memory(dw, depth)
            ports[n] = mems[n].get_port(has_re=True, mode=READ_FIRST)
            self.specials += ports[n]
        self.mems = mems

        # Connect Memory ports.
        cases = {}
        for n, port in enumerate(ports):
            self.comb += port.re.eq(read)
            self.comb += port.adr.eq(length[int(math.log2(dw//8)):])
            cases[n] = [rd_data.eq(port.dat_r)]

        self.comb += Case(rd_slot, cases)

        # Endianness Handling.
        self.comb += source.data.eq({"big" : reverse_bytes(rd_data), "little": rd_data}[endianness])

# MAC SRAM -----------------------------------------------------------------------------------------

class LiteEthMACSRAM(Module, AutoCSR):
    def __init__(self, dw, depth, nrxslots, ntxslots, endianness, timestamp=None, with_eth_pcie=True):
        self.submodules.writer = LiteEthMACSRAMWriter(dw, depth, nrxslots, endianness, timestamp)
        self.submodules.reader = LiteEthMACSRAMReader(dw, depth, ntxslots, endianness, timestamp)
        if with_eth_pcie:
            self.rx_pcie_irq = self.writer.pcie_irq
            self.tx_pcie_irq = self.reader.pcie_irq
        else:
            self.submodules.ev = SharedIRQ(self.writer.ev, self.reader.ev)
        self.sink, self.source = self.writer.sink, self.reader.source
