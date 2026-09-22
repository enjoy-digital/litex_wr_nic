#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex_wr_nic.gateware.sfp_eeprom import (
    I2CEEPROMEmulator,
    SFPEEPROMCache,
    SFPEEPROMReader,
)

# Bit-Bang Master Model ----------------------------------------------------------------------------

class BitBangMaster:
    """Drive the emulator like WRPC's bb_i2c: a few cycles per bus state."""
    def __init__(self, emulator, hold=6):
        self.emulator = emulator
        self.hold = hold

    def _set(self, scl, sda):
        yield self.emulator.scl_i.eq(scl)
        yield self.emulator.sda_i.eq(sda)
        for _ in range(self.hold):
            yield

    def start(self):
        yield from self._set(1, 1)
        yield from self._set(1, 0)
        yield from self._set(0, 0)

    def stop(self):
        yield from self._set(0, 0)
        yield from self._set(1, 0)
        yield from self._set(1, 1)

    def put(self, value):
        for shift in range(7, -1, -1):
            bit = (value >> shift) & 1
            yield from self._set(0, bit)
            yield from self._set(1, bit)
            yield from self._set(0, bit)
        yield from self._set(0, 1)
        yield from self._set(1, 1)
        ack = not (yield self.emulator.sda_o)
        yield from self._set(0, 1)
        return ack

    def get(self, last):
        value = 0
        yield from self._set(0, 1)
        for _ in range(8):
            yield from self._set(1, 1)
            value = (value << 1) | (yield self.emulator.sda_o)
            yield from self._set(0, 1)
        yield from self._set(0, int(last))
        yield from self._set(1, int(last))
        yield from self._set(0, 1)
        return value

# Emulator Tests -----------------------------------------------------------------------------------

def test_emulator_serves_pointer_and_sequential_reads_like_wrpc():
    dut = Module()
    dut.submodules.emulator = emulator = I2CEEPROMEmulator()
    image = [(3*i + 7) & 0xff for i in range(256)]
    dut.sync += emulator.mem_dat.eq(Array(image)[emulator.mem_adr])
    master = BitBangMaster(emulator)

    def stimulus():
        # sfp_read_i2c: pointer write, repeated start, one byte with NACK, stop.
        yield from master.start()
        assert (yield from master.put(0x50 << 1))
        assert (yield from master.put(40))
        yield from master.start()
        assert (yield from master.put((0x50 << 1) | 1))
        assert (yield from master.get(last=True)) == image[40]
        yield from master.stop()
        # Then a current-address read continues from the next byte.
        yield from master.start()
        assert (yield from master.put((0x50 << 1) | 1))
        got = []
        for i in range(5):
            got.append((yield from master.get(last=(i == 4))))
        yield from master.stop()
        assert got == image[41:46]
        # Another device address is ignored, and reads wrap at the end.
        yield from master.start()
        assert not (yield from master.put(0x51 << 1))
        yield from master.stop()
        yield from master.start()
        assert (yield from master.put(0x50 << 1))
        assert (yield from master.put(254))
        yield from master.start()
        assert (yield from master.put((0x50 << 1) | 1))
        got = []
        for i in range(3):
            got.append((yield from master.get(last=(i == 2))))
        yield from master.stop()
        assert got == [image[254], image[255], image[0]]
        assert (yield emulator.active) == 0

    run_simulation(dut, stimulus())

# Reader Tests -------------------------------------------------------------------------------------

class EEPROMSlaveModel:
    """Respond to the reader's I2C master as a 24Cxx at address 0x50."""
    def __init__(self, reader, image, address=0x50):
        self.reader  = reader
        self.image   = image
        self.address = address
        self.scl = 1
        self.sda = 1
        self.bits = []
        self.state = "idle"
        self.pointer = 0
        self.byte = 0
        self.drive = 1
        self.transactions = 0

    def sample(self):
        scl = (yield self.reader.scl_o)
        sda = (yield self.reader.sda_o)
        if scl and self.scl:
            if self.sda and not sda:
                self.state, self.bits = "address", []
            elif sda and not self.sda:
                self.state = "idle"
        if scl and not self.scl:                     # rising: master bit valid
            if self.state in ("address", "pointer"):
                self.bits.append(sda)
            elif self.state == "read-ack":
                self.master_ack = not sda
        if not scl and self.scl:                     # falling: update our output
            if self.state in ("address", "pointer") and len(self.bits) == 8:
                value = int("".join(map(str, self.bits)), 2)
                self.bits = []
                if self.state == "address":
                    if value >> 1 == self.address:
                        self.drive = 0               # ACK
                        self.next = "read-data" if value & 1 else "pointer"
                        if value & 1:
                            self.transactions += 1
                    else:
                        self.drive, self.next = 1, "idle"
                else:
                    self.pointer, self.drive, self.next = value, 0, "pointer"
                self.state = "ack"
            elif self.state == "ack":
                self.drive = 1
                self.state = self.next
                if self.state == "read-data":
                    self.byte, self.bit = self.image[self.pointer % 256], 7
                    self.drive = (self.byte >> 7) & 1
            elif self.state == "read-data":
                self.bit -= 1
                if self.bit >= 0:
                    self.drive = (self.byte >> self.bit) & 1
                else:
                    self.drive = 1
                    self.pointer += 1
                    self.state = "read-ack"
            elif self.state == "read-ack":
                if self.master_ack:
                    self.byte, self.bit = self.image[self.pointer % 256], 7
                    self.drive = (self.byte >> 7) & 1
                    self.state = "read-data"
                else:
                    self.state = "idle"
        self.scl, self.sda = scl, sda
        yield self.reader.sda_i.eq(self.drive & sda)


def test_reader_copies_the_eeprom_and_retries_after_nack():
    dut   = Module()
    image = [(5*i + 1) & 0xff for i in range(256)]
    dut.submodules.cache = cache = SFPEEPROMCache(1e6, i2c_freq=250e3, select_cycles=4, retry_cycles=400)
    reader = cache.reader
    dut.comb += reader.grant.eq(reader.request)
    slave = EEPROMSlaveModel(reader, image)

    def stimulus():
        for _ in range(30000):
            yield from slave.sample()
            yield
            if (yield reader.valid):
                break
        else:
            raise AssertionError("EEPROM copy did not complete")
        assert (yield reader._reads.status) == 1
        assert slave.transactions == 1
        # The copy is readable by the host through the cache window.
        for address in (0, 1, 100, 255):
            yield cache._address.storage.eq(address)
            yield
            yield
            assert (yield cache._data.status) == image[address]
        # A refresh with a silent bus fails, invalidates, and retries later.
        slave.address = 0x57
        yield cache.reader._control.fields.refresh.eq(1)
        yield
        yield cache.reader._control.fields.refresh.eq(0)
        for _ in range(3000):
            yield from slave.sample()
            yield
        assert (yield reader.valid) == 0
        assert (yield reader._errors.status) >= 1
        errors = (yield reader._errors.status)
        for _ in range(3000):
            yield from slave.sample()
            yield
        assert (yield reader._errors.status) > errors

    run_simulation(dut, stimulus())
