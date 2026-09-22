#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.soc.interconnect.csr import CSRField, CSRStatus, CSRStorage

from litex.soc.cores.i2c import I2CMasterMachine

# SFP EEPROM ---------------------------------------------------------------------------------------
#
# WRPC identifies the SFP module by bit-banging the 256-byte EEPROM at I2C
# address 0x50 (SFF-8472 A0h) from its own GPIO. On boards where that EEPROM
# shares an I2C bus with a clock actuator, the firmware's transfers, which
# cannot be stalled, would collide with the actuator. The reader below copies
# the EEPROM into a memory with a hardware I2C master while it owns the bus,
# and the emulator serves that copy to the firmware as a 24Cxx-style device.

SFP_EEPROM_ADDRESS = 0x50
SFP_EEPROM_SIZE    = 256


class I2CEEPROMEmulator(LiteXModule):
    """Serve a memory as a byte-addressed I2C EEPROM to a bit-banged master.

    ``scl_i``/``sda_i`` are the line levels driven by the master and
    ``sda_o`` is the emulator's open-drain intent (0 drives the line low).
    A write transfer sets the address pointer and its data bytes are
    acknowledged and discarded; reads return the memory from the pointer with
    auto-increment, including current-address reads. The lines are sampled at
    the system clock, so the master must be much slower than it. The memory
    is read through ``mem_adr``/``mem_dat`` (one cycle of read latency) while
    ``active`` is high.
    """
    def __init__(self, address=SFP_EEPROM_ADDRESS, size=SFP_EEPROM_SIZE):
        self.scl_i   = Signal(reset=1)
        self.sda_i   = Signal(reset=1)
        self.sda_o   = Signal(reset=1)
        self.active  = Signal()
        self.mem_adr = Signal(max=size)
        self.mem_dat = Signal(8)

        # # #

        scl   = Signal(reset=1)
        sda   = Signal(reset=1)
        scl_d = Signal(reset=1)
        sda_d = Signal(reset=1)
        self.specials += [
            MultiReg(self.scl_i, scl, reset=1),
            MultiReg(self.sda_i, sda, reset=1),
        ]
        self.sync += [scl_d.eq(scl), sda_d.eq(sda)]
        scl_rise = Signal()
        scl_fall = Signal()
        start    = Signal()
        stop     = Signal()
        self.comb += [
            scl_rise.eq(scl & ~scl_d),
            scl_fall.eq(~scl & scl_d),
            start.eq(scl & scl_d & sda_d & ~sda),
            stop.eq(scl & scl_d & ~sda_d & sda),
        ]

        shift   = Signal(8)
        bits    = Signal(4)
        pointer = Signal(max=size)
        reading = Signal()
        self.comb += self.mem_adr.eq(pointer)

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.comb += self.active.eq(~fsm.ongoing("IDLE"))
        # START from any state (re)synchronizes on the device address.
        fsm.act("IDLE",
            NextValue(self.sda_o, 1),
            If(start,
                NextValue(bits, 0),
                NextState("ADDRESS"),
            ),
        )
        fsm.act("ADDRESS",
            If(scl_rise,
                NextValue(shift, Cat(sda, shift[:7])),
                NextValue(bits, bits + 1),
            ),
            If(scl_fall & (bits == 8),
                If(shift[1:] == address,
                    NextValue(self.sda_o, 0),
                    NextValue(reading, shift[0]),
                    NextState("ACK-ADDRESS"),
                ).Else(
                    NextState("IDLE"),
                ),
            ),
        )
        fsm.act("ACK-ADDRESS",
            If(scl_fall,
                NextValue(bits, 0),
                If(reading,
                    NextValue(shift, self.mem_dat),
                    NextValue(self.sda_o, self.mem_dat[7]),
                    NextState("DATA-READ"),
                ).Else(
                    NextValue(self.sda_o, 1),
                    NextState("POINTER"),
                ),
            ),
        )
        fsm.act("POINTER",
            If(scl_rise,
                NextValue(shift, Cat(sda, shift[:7])),
                NextValue(bits, bits + 1),
            ),
            If(scl_fall & (bits == 8),
                NextValue(pointer, shift),
                NextValue(self.sda_o, 0),
                NextState("ACK-WRITE"),
            ),
        )
        fsm.act("ACK-WRITE",
            If(scl_fall,
                NextValue(bits, 0),
                NextValue(self.sda_o, 1),
                NextState("DATA-WRITE"),
            ),
        )
        # Data bytes of a write are acknowledged and discarded: the image is
        # read-only from the master's point of view.
        fsm.act("DATA-WRITE",
            If(scl_rise,
                NextValue(bits, bits + 1),
            ),
            If(scl_fall & (bits == 8),
                NextValue(self.sda_o, 0),
                NextState("ACK-WRITE"),
            ),
        )
        fsm.act("DATA-READ",
            If(scl_fall,
                NextValue(bits, bits + 1),
                NextValue(shift, Cat(0, shift[:7])),
                NextValue(self.sda_o, shift[6]),
                If(bits == 7,
                    NextValue(self.sda_o, 1),
                    NextValue(pointer, pointer + 1),
                    NextState("ACK-READ"),
                ),
            ),
        )
        # The master acknowledges to continue; a NACK ends the transfer.
        fsm.act("ACK-READ",
            If(scl_rise,
                NextValue(reading, ~sda),
            ),
            If(scl_fall,
                NextValue(bits, 0),
                If(reading,
                    NextValue(shift, self.mem_dat),
                    NextValue(self.sda_o, self.mem_dat[7]),
                    NextState("DATA-READ"),
                ).Else(
                    NextState("IDLE"),
                ),
            ),
        )
        for state in fsm.actions:
            if state != "IDLE":
                fsm.act(state,
                    If(start,
                        NextValue(bits, 0),
                        NextValue(self.sda_o, 1),
                        NextState("ADDRESS"),
                    ).Elif(stop,
                        NextValue(self.sda_o, 1),
                        NextState("IDLE"),
                    ),
                )


class SFPEEPROMReader(LiteXModule):
    """Copy an SFP EEPROM into a memory with a hardware I2C master.

    A read is started by ``refresh`` (host or automatic) once ``grant`` is
    high: ``request`` asks the bus owner for the bus and ``busy`` holds it.
    ``valid`` reports a complete acknowledged copy; a NACK invalidates it and
    a new attempt follows ``retry_cycles`` later while ``auto`` is set, so a
    module inserted later is picked up. ``select_cycles`` lets an upstream
    multiplexer settle before START.
    """
    def __init__(self, port, sys_clk_freq, i2c_freq=400e3, address=SFP_EEPROM_ADDRESS,
        size=SFP_EEPROM_SIZE, select_cycles=64, retry_cycles=None):
        if retry_cycles is None:
            retry_cycles = int(sys_clk_freq) # One second.
        self.request = Signal()
        self.grant   = Signal()
        self.busy    = Signal()
        self.valid   = Signal()
        self.scl_o   = Signal(reset=1)
        self.sda_o   = Signal(reset=1)
        self.sda_i   = Signal()

        self._control = CSRStorage(fields=[
            CSRField("refresh", size=1, pulse=True, description="Read the EEPROM now."),
            CSRField("auto",    size=1, offset=1, reset=1,
                description="Read at start-up and retry every second until a copy is valid."),
        ])
        self._status = CSRStatus(fields=[
            CSRField("valid", size=1, description="The memory holds an acknowledged EEPROM copy."),
            CSRField("busy",  size=1, description="A read is in progress."),
        ])
        self._reads  = CSRStatus(32, description="Completed EEPROM reads.")
        self._errors = CSRStatus(32, description="Reads aborted by a missing acknowledge.")

        # # #

        self.i2c = i2c = I2CMasterMachine(clock_width=16)
        self.comb += [
            i2c.cg.load.eq(int(sys_clk_freq/(2*i2c_freq)) - 1),
            self.scl_o.eq(i2c.scl_o),
            self.sda_o.eq(i2c.sda_o),
            i2c.sda_i.eq(self.sda_i),
        ]

        count   = Signal(max=size + 1)
        failed  = Signal()
        pending = Signal(reset=1) # Read at start-up.
        self.retry = retry = WaitTimer(retry_cycles)
        self.sync += [
            If(self._control.fields.refresh, pending.eq(1)),
            If(retry.done & self._control.fields.auto & ~self.valid, pending.eq(1)),
        ]
        self.comb += [
            port.adr.eq(count),
            port.we.eq(0),
            self._status.fields.valid.eq(self.valid),
            self._status.fields.busy.eq(self.busy),
        ]

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.comb += self.busy.eq(~fsm.ongoing("IDLE") & ~fsm.ongoing("REQUEST"))
        fsm.act("IDLE",
            retry.wait.eq(~pending),
            If(pending,
                NextState("REQUEST"),
            ),
        )
        fsm.act("REQUEST",
            self.request.eq(1),
            If(self.grant,
                NextValue(pending, 0),
                NextValue(count, 0),
                NextValue(failed, 0),
                NextState("SELECT"),
            ),
        )
        fsm.delayed_enter("SELECT", "START", select_cycles)
        # Pointer write: START, address+W, 0x00; then repeated START, address+R.
        fsm.act("START",
            self.request.eq(1),
            i2c.start.eq(1),
            NextState("START-WAIT"),
        )
        fsm.act("START-WAIT",
            self.request.eq(1),
            If(i2c.idle,
                NextValue(i2c.data, address << 1),
                NextState("WRITE-ADDRESS"),
            ),
        )
        fsm.act("WRITE-ADDRESS",
            self.request.eq(1),
            i2c.write.eq(1),
            NextState("WRITE-ADDRESS-WAIT"),
        )
        fsm.act("WRITE-ADDRESS-WAIT",
            self.request.eq(1),
            If(i2c.idle,
                If(~i2c.ack,
                    NextValue(failed, 1),
                    NextState("STOP"),
                ).Else(
                    NextValue(i2c.data, 0),
                    NextState("WRITE-POINTER"),
                ),
            ),
        )
        fsm.act("WRITE-POINTER",
            self.request.eq(1),
            i2c.write.eq(1),
            NextState("WRITE-POINTER-WAIT"),
        )
        fsm.act("WRITE-POINTER-WAIT",
            self.request.eq(1),
            If(i2c.idle,
                If(~i2c.ack,
                    NextValue(failed, 1),
                    NextState("STOP"),
                ).Else(
                    NextState("RESTART"),
                ),
            ),
        )
        fsm.act("RESTART",
            self.request.eq(1),
            i2c.start.eq(1),
            NextState("RESTART-WAIT"),
        )
        fsm.act("RESTART-WAIT",
            self.request.eq(1),
            If(i2c.idle,
                NextValue(i2c.data, (address << 1) | 1),
                NextState("WRITE-READ-ADDRESS"),
            ),
        )
        fsm.act("WRITE-READ-ADDRESS",
            self.request.eq(1),
            i2c.write.eq(1),
            NextState("WRITE-READ-ADDRESS-WAIT"),
        )
        fsm.act("WRITE-READ-ADDRESS-WAIT",
            self.request.eq(1),
            If(i2c.idle,
                If(~i2c.ack,
                    NextValue(failed, 1),
                    NextState("STOP"),
                ).Else(
                    NextState("READ-SETUP"),
                ),
            ),
        )
        # i2c.ack is the machine's own register: it samples the acknowledge of
        # a write into it and sends it after a read. Set it between transfers,
        # never combinationally, so the two uses cannot fight.
        fsm.act("READ-SETUP",
            self.request.eq(1),
            NextValue(i2c.ack, count != size - 1), # NACK the last byte.
            NextState("READ"),
        )
        fsm.act("READ",
            self.request.eq(1),
            i2c.read.eq(1),
            NextState("READ-WAIT"),
        )
        fsm.act("READ-WAIT",
            self.request.eq(1),
            If(i2c.idle,
                port.we.eq(1),
                NextValue(count, count + 1),
                If(count == size - 1,
                    NextState("STOP"),
                ).Else(
                    NextState("READ-SETUP"),
                ),
            ),
        )
        fsm.act("STOP",
            self.request.eq(1),
            i2c.stop.eq(1),
            NextState("STOP-WAIT"),
        )
        fsm.act("STOP-WAIT",
            self.request.eq(1),
            If(i2c.idle,
                If(failed,
                    NextValue(self.valid, 0),
                    NextValue(self._errors.status, self._errors.status + 1),
                ).Else(
                    NextValue(self.valid, 1),
                    NextValue(self._reads.status, self._reads.status + 1),
                ),
                NextState("RELEASE"),
            ),
        )
        # Keep the bus through its free time before handing it back.
        fsm.delayed_enter("RELEASE", "IDLE", select_cycles)
        self.comb += port.dat_w.eq(i2c.data)


class SFPEEPROMCache(LiteXModule):
    """Reader and emulator around one 256-byte memory, with a host window.

    The host reads the copy through the emulator's port between firmware
    transfers; ``data`` is valid a cycle after ``address`` is written.
    """
    def __init__(self, sys_clk_freq, **kwargs):
        mem = Memory(8, SFP_EEPROM_SIZE)
        write_port = mem.get_port(write_capable=True)
        read_port  = mem.get_port()
        self.specials += mem, write_port, read_port
        self.reader   = reader   = SFPEEPROMReader(write_port, sys_clk_freq, **kwargs)
        self.emulator = emulator = I2CEEPROMEmulator()

        self._address = CSRStorage(8, description="Byte address read through data.")
        self._data    = CSRStatus(8, description="EEPROM copy byte at address.")
        self.comb += [
            read_port.adr.eq(Mux(emulator.active, emulator.mem_adr, self._address.storage)),
            emulator.mem_dat.eq(read_port.dat_r),
            self._data.status.eq(read_port.dat_r),
        ]
