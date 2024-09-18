import struct
import io


"""
This is an implementation of the BIT file format from
https://github.com/mfischer/fpgadev-zynq/blob/master/top/python/bit_to_zynq_bin.py
and of the BIN bitstream format for 7 series from
https://www.xilinx.com/support/documentation/user_guides/ug470_7Series_Config.pdf
and ideas from:

F. Benz, A. Seffrin, S. A. Huss, "Bil: A tool-chain for bitstream
reverse-engineering", Proc. 22nd Int. Conf. Field Program. Logic Appl., pp.
735-738, 2012.
https://doi.org/10.1109/FPL.2012.6339165
https://github.com/florianbenz/bil

Jean-Baptiste Note, Éric Rannaud, "From the bitstream to the netlist",
Proceedings of the 16th international ACM/SIGDA symposium on Field programmable
gate arrays, February 24-26, 2008, Monterey, California, USA
https://doi.org/10.1145/1344671.1344729
https://github.com/djn3m0/debit
"""


def flip32(data):
    sl = struct.Struct("<I")
    sb = struct.Struct(">I")
    b = memoryview(data)
    d = bytearray(len(data))
    for offset in range(0, len(data), sl.size):
         sb.pack_into(d, offset, *sl.unpack_from(b, offset))
    return d


class Parser:
    def __init__(self, stream):
        self.stream = stream

    def handle_bit(self):
        a, = struct.unpack(">H", self.stream.read(2))
        if a != 9:
            raise ValueError("Missing <0009> header, not a bit file")
        unk = self.stream.read(a)  # unknown data
        b, = struct.unpack(">H", self.stream.read(2))
        if b != 1:
            raise ValueError("Missing <0001> header, not a bit file")
        self.handle_bitstart(a, unk, b)

        while True:
            key = self.stream.read(1)
            if not key:
                break
            self.handle_keystart(key)
            if key == b"e":
                length, = struct.unpack(">I", self.stream.read(4))
                self.handle_binstart(length)
                self.handle_bin(end_at=self.stream.tell() + length)
            elif key in b"abcd":
                data = self.stream.read(*struct.unpack(">H",
                    self.stream.read(2)))
                self.handle_meta(key, data)
            else:
                print("Unknown key: {}: {}".format(key, d))

    def handle_bitstart(self, a, unk, b):
        pass

    def handle_keystart(self, key):
        pass

    def handle_meta(self, key, data):
        assert data.endswith(b"\x00")
        data = data[:-1].decode()
        name = {
                b"a": "Design",
                b"b": "Part name",
                b"c": "Date",
                b"d": "Time"
                }[key]
        print("{}: {}".format(name, data))

    def handle_binstart(self, length):
        print("Bitstream payload length: {:#x}".format(length))

    def handle_bin(self, end_at=None):
        sync = b""
        while not sync.endswith(b"\xaa\x99\x55\x66"):
            sync += self.stream.read(1)
        self.handle_sync(sync)
        while True:
            if end_at is not None and self.stream.tell() >= end_at:
                assert self.stream.tell() == end_at
                break
            hdr = self.stream.read(4)
            if len(hdr) != 4:
                assert end is None
                assert len(hdr) == 0
                break
            hdr, = struct.unpack(">I", hdr)
            typ = hdr >> 29
            if typ == 1:
                self.handle_type1(hdr)
            elif typ == 2:
                self.handle_type2(hdr)
            else:
                raise ValueError("no such packet type", hdr)
        self.handle_end()

    def handle_sync(self, sync):
        pass

    def handle_end(self):
        pass

    def handle_type1(self, hdr):
        op = (hdr >> 27) & 0x3
        self.addr = (hdr >> 13) & 0x7ff
        assert self.addr == self.addr & 0x1f
        length = hdr & 0x7ff
        payload = self.stream.read(length * 4)
        assert len(payload) == length * 4
        self.handle_op(op, hdr, payload)

    def handle_type2(self, hdr):
        op = (hdr >> 27) & 0x3
        length = hdr & 0x7ffffff
        payload = self.stream.read(length * 4)
        assert len(payload) == length * 4
        self.handle_op(op, hdr, payload)

    def handle_op(self, op, hdr, payload):
        assert op != 3
        if op == 0:
            self.handle_nop(hdr, payload)
        elif op == 1:
            self.handle_read(hdr, payload)
        elif op == 2:
            self.handle_write(hdr, payload)

    def handle_nop(self, hdr, payload):
        pass

    def handle_read(self, hdr, payload):
        pass

    def handle_write(self, hdr, payload):
        pass


class Rewriter(Parser):
    def __init__(self, read, write):
        self.output = write
        return super().__init__(read)

    def handle_bitstart(self, a, unk, b):
        self.output.write(struct.pack(">H", a))
        self.output.write(unk)
        self.output.write(struct.pack(">H", b))

    def handle_keystart(self, key):
        self.output.write(struct.pack(">c", key))

    def handle_meta(self, key, data):
        super().handle_meta(key, data)
        self.output.write(struct.pack(">H", len(data)))
        self.output.write(data)

    def handle_binstart(self, length):
        super().handle_binstart(length)
        self.output.write(struct.pack(">I", length))

    def handle_sync(self, sync):
        self.output.write(sync)

    def handle_end(self):
        pass

    def handle_nop(self, hdr, payload):
        self.emit_packet(hdr, payload)

    def handle_write(self, hdr, payload):
        self.emit_packet(hdr, payload)

    def handle_read(self, hdr, payload):
        self.emit_packet(hdr, payload)

    def emit_packet(self, hdr, payload):
        self.output.write(struct.pack(">I", hdr))
        self.output.write(payload)


registers = list(
    "CRC FAR FDRI FDRO CMD CTL0 MASK STAT "
    "LOUT COR0 MFWR CBC IDCODE AXSS COR1 _ "
    "WBSTAR TIMER _ _ _ _ BOOTSTS _ "
    "CTL1 _ _ _ _ _ _ BSPI".split())

assert registers[0b11000] == "CTL1"
assert registers[0b10110] == "BOOTSTS"
assert registers[0b11111] == "BSPI"
assert registers[0b01100] == "IDCODE"


class Squeeze(Rewriter):
    idmap = {
            0x0362C093: 0x0362D093,
            0x0362D093: 0x0362D093,
            }

    def handle_write(self, hdr, payload):
        if self.addr == registers.index("CRC"):
            crc, = struct.unpack(">I", payload)
            print(hex(crc))
            self.addr = None
            # drop crc verification packets
            return
        elif self.addr == registers.index("IDCODE"):
            idcode, = struct.unpack(">I", payload)
            payload = struct.pack(">I", self.idmap.get(idcode, idcode))
        self.emit_packet(hdr, payload)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description="Xilinx Bitstream parser and rewriter")
    parser.add_argument("bitfile", metavar="BITFILE",
                        help="Input bit file name")
    parser.add_argument("outfile", metavar="BINFILE",
                        help="Output bin file name")
    args = parser.parse_args()

    read = io.BytesIO(open(args.bitfile, "rb").read())

    Parser(read).handle_bit()
    read.seek(0)

    write = io.BytesIO()
    Rewriter(read, write).handle_bit()
    assert read.getbuffer() == write.getbuffer()
    read.seek(0)
    write.seek(0)

    Squeeze(read, write).handle_bit()
    with open(args.outfile, "wb") as f:
        f.write(write.getbuffer())
