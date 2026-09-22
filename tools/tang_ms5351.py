#!/usr/bin/env python3
"""Inspect or restore the Tang Mega 138K Pro dock MS5351 PLL0 through the host I2C path.

The WR main-clock tuner assumes the documented dock configuration: PLLA at 25 MHz x 36
(900 MHz) with MS0/MS1 = /9 for the 100 MHz differential SerDes reference.

The MS5351 aborts a transfer whose clock edges are more than a few milliseconds apart,
so the bit-bang states are sent as fixed-address UARTBone bursts on the FPGA UART.
Stop any litex_server using that UART first; this tool opens it exclusively.
"""

import argparse
import time

from litex.tools.remote.comm_uart import CommUART

MS5351_ADDRESS   = 0x60
MS5351_CHANNEL   = 3       # Dock I2C multiplexer channel of PLL0.
MS5351_PLLA      = 26
MS5351_MS0       = 42
MS5351_MS1       = 50
MS5351_PLL_RESET = 177
MS5351_OUTPUTS   = 3
EXPECTED_PLL_MULTIPLIER = 36
EXPECTED_MS_DIVIDER     = 9


class BitBangI2C:
    """Host I2C on the LiteX bit-bang core (i2c_w/i2c_r CSRs), bursting bus states."""
    def __init__(self, bus):
        self.bus = bus

    def _sequence(self, states):
        self.bus.write(self.bus.regs.i2c_w.addr,
            [int(scl) | (1 << 1) | (int(sda) << 2) for scl, sda in states], burst="fixed")

    def _set(self, scl, sda):
        self._sequence([(scl, sda)])

    def _high(self, sda):
        self._set(1, sda)
        deadline = time.monotonic() + 0.1
        while True:
            value = self.bus.regs.i2c_r.read()
            if value & 2:
                return value & 1
            if time.monotonic() > deadline:
                raise TimeoutError("I2C SCL held low")

    def start(self):
        self._high(1)
        self._sequence([(1, 0), (0, 0)])

    def stop(self):
        self._set(0, 0)
        self._high(0)
        self._set(1, 1)

    def write(self, value):
        states = []
        for shift in range(7, -1, -1):
            bit = (value >> shift) & 1
            states += [(0, bit), (1, bit), (0, bit)]
        self._sequence(states + [(0, 1)])
        ack = not self._high(1)
        self._set(0, 1)
        return ack

    def read(self, ack):
        value = 0
        self._set(0, 1)
        for _ in range(8):
            value = (value << 1) | self._high(1)
            self._set(0, 1)
        self._sequence([(0, not ack), (1, not ack), (0, not ack), (0, 1)])
        return value

    def read_regs(self, address, register, length):
        self.start()
        try:
            for value in (address << 1, register):
                if not self.write(value):
                    raise RuntimeError(f"I2C NACK writing {value:#x}")
            self.start()
            if not self.write((address << 1) | 1):
                raise RuntimeError("I2C NACK on repeated start")
            return bytes(self.read(i < length - 1) for i in range(length))
        finally:
            self.stop()

    def write_regs(self, address, register, data):
        self.start()
        try:
            for value in (address << 1, register, *data):
                if not self.write(value):
                    raise RuntimeError(f"I2C NACK writing {value:#x}")
        finally:
            self.stop()


def parameters(p1, p2=0, p3=1):
    """Encode the eight PLL/MultiSynth parameter registers."""
    return bytes([p3 >> 8 & 255, p3 & 255, p1 >> 16 & 3, p1 >> 8 & 255, p1 & 255,
                  (p3 >> 16 << 4) | (p2 >> 16), p2 >> 8 & 255, p2 & 255])


def decode(regs):
    p3 = ((regs[5] >> 4) << 16) | (regs[0] << 8) | regs[1]
    p1 = ((regs[2] & 3) << 16) | (regs[3] << 8) | regs[4]
    p2 = ((regs[5] & 15) << 16) | (regs[6] << 8) | regs[7]
    return p1, p2, p3


def integer_setting(p1, p2, p3):
    """Return the integer multiplier/divider of P1/P2/P3, or None if fractional."""
    if p2 != 0 or (p1 + 512) % 128:
        return None
    return (p1 + 512) // 128


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--csr-csv", default="build/tang_mega_138k_pro_wr/csr.csv")
    parser.add_argument("--uart-port", required=True, help="FPGA UART device (UARTBone).")
    parser.add_argument("--uart-baudrate", type=int, default=115200)
    parser.add_argument("--restore", action="store_true",
        help="Program PLLA = x36 and MS0/MS1 = /9, then reset the PLL.")
    args = parser.parse_args()

    bus = CommUART(args.uart_port, args.uart_baudrate, csr_csv=args.csr_csv)
    bus.open()
    try:
        # Release the bus from the WR tuner and select the PLL0 channel.
        tuner_enabled = bus.regs.main_tuning_control.read()
        bus.regs.main_tuning_control.write(0)
        time.sleep(0.01)
        bus.regs.main_i2c_sel.write(MS5351_CHANNEL)
        i2c  = BitBangI2C(bus)
        regs = i2c.read_regs(MS5351_ADDRESS, 0, 66)
        if args.restore:
            outputs = regs[MS5351_OUTPUTS]
            i2c.write_regs(MS5351_ADDRESS, MS5351_OUTPUTS, [outputs | 0x03])
            i2c.write_regs(MS5351_ADDRESS, MS5351_PLLA, parameters(128*EXPECTED_PLL_MULTIPLIER - 512))
            i2c.write_regs(MS5351_ADDRESS, MS5351_MS0,  parameters(128*EXPECTED_MS_DIVIDER - 512))
            i2c.write_regs(MS5351_ADDRESS, MS5351_MS1,  parameters(128*EXPECTED_MS_DIVIDER - 512))
            i2c.write_regs(MS5351_ADDRESS, MS5351_PLL_RESET, [0x20])
            time.sleep(0.01)
            i2c.write_regs(MS5351_ADDRESS, MS5351_OUTPUTS, [outputs & ~0x03])
            time.sleep(0.5)
            regs = i2c.read_regs(MS5351_ADDRESS, 0, 66)
        ok = True
        for name, base, expected in (("PLLA", MS5351_PLLA, EXPECTED_PLL_MULTIPLIER),
                                     ("MS0", MS5351_MS0, EXPECTED_MS_DIVIDER),
                                     ("MS1", MS5351_MS1, EXPECTED_MS_DIVIDER)):
            p1, p2, p3 = decode(regs[base:base + 8])
            value = integer_setting(p1, p2, p3)
            if name == "PLLA":
                # The tuner keeps P1 at x36 or x35 + 127/128 and spans P2/P3.
                value = (p1 + 512 + 1) // 128 if p3 == (1 << 20) - 1 else value
            status = "ok" if value == expected else f"expected x{expected}" if name == "PLLA" else f"expected /{expected}"
            ok = ok and value == expected
            print(f"{name:4s} P1={p1:<6d} P2={p2:<7d} P3={p3:<7d} -> {value} ({status})")
        print(f"Outputs enable register: {regs[MS5351_OUTPUTS]:#04x} (bits 0/1 must be clear)")
        print("MS5351 PLL0 configuration matches the WR tuner assumptions." if ok else
              "MS5351 PLL0 configuration differs: run with --restore or use the dock PLL console.")
    finally:
        bus.regs.main_i2c_sel.write(0)
        bus.regs.main_tuning_control.write(tuner_enabled)
        bus.close()


if __name__ == "__main__":
    main()
