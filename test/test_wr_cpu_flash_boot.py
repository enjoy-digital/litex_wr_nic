#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen.sim import passive, run_simulation

from litex_wr_nic.gateware.wr_cpu import WRCPUFlashBoot
from litex_wr_nic.wr_boot import build_boot_image

# Flash Boot Simulation ----------------------------------------------------------------------------

def simulate_boot(image, startup_clocks=0, memory_latency=0):
    dut = WRCPUFlashBoot(
        sys_clk_freq     = 100,
        spi_clk_freq     = 25,
        memory_wait      = 0,
        wishbone_timeout = 32,
    )
    writes = []
    result = {}

    @passive
    def flash_model():
        previous_clk = 0
        rising_edges = 0
        total_edges  = 0
        command      = 0
        while True:
            cs_n = yield dut.cs_n
            clk  = yield dut.clk
            if cs_n:
                rising_edges = 0
                command      = 0
                yield dut.miso.eq(0)
            else:
                # The first 32 clocks carry READ (0x03) and the 24-bit address.
                if clk == 0:
                    bit_index = rising_edges - 32
                    if 0 <= bit_index < 8*len(image):
                        byte = image[bit_index // 8]
                        yield dut.miso.eq((byte >> (7 - bit_index % 8)) & 1)
                    else:
                        yield dut.miso.eq(0)
                if not previous_clk and clk and total_edges >= startup_clocks:
                    if rising_edges < 32:
                        command = (command << 1) | (yield dut.mosi)
                        if rising_edges == 31:
                            assert command == 0x032f0000, hex(command)
                    rising_edges += 1
            if not previous_clk and clk:
                total_edges += 1
            previous_clk = clk
            yield

    @passive
    def memory_model():
        active    = False
        responded = False
        remaining = 0
        request   = None
        while True:
            cyc = yield dut.bus.cyc
            stb = yield dut.bus.stb
            yield dut.bus.ack.eq(0)
            if not cyc:
                active = False
            elif not active and stb:
                request   = ((yield dut.bus.adr), (yield dut.bus.dat_w), (yield dut.bus.sel))
                active    = True
                responded = False
                remaining = memory_latency
            if cyc and active and not responded:
                assert stb, "Classic Wishbone STB dropped before the memory acknowledged"
                assert request == ((yield dut.bus.adr), (yield dut.bus.dat_w), (yield dut.bus.sel))
                if remaining == 0:
                    writes.append(request)
                    yield dut.bus.ack.eq(1)
                    responded = True
                else:
                    remaining -= 1
            yield

    def monitor():
        for _ in range(20_000):
            if (yield dut.ready) or not (yield dut.owner):
                result["ready"] = yield dut.ready
                result["error"] = yield dut._error.status
                registers = [
                    dut.ready,
                    dut.owner,
                    dut._error.status,
                    dut._progress.status,
                    dut._length.status,
                    dut._expected_crc.status,
                    dut._actual_crc.status,
                ]
                snapshot = []
                for reg in registers:
                    snapshot.append((yield reg))
                write_count = len(writes)
                # Completion is permanent until reset. In particular, empty
                # Migen FSM actions must not fall through to the header parser.
                for _ in range(2_000):
                    yield
                    for reg, value in zip(registers, snapshot):
                        assert (yield reg) == value, "Boot completion changed after termination"
                    assert (yield dut.bus.cyc) == 0
                    assert (yield dut.bus.stb) == 0
                    assert len(writes) == write_count
                assert (yield dut.cs_n) == 1 and (yield dut.clk) == 0, (
                    (yield dut.spi_fsm.state), dut.spi_fsm.encoding, (yield dut.owner),
                )
                return
            yield
        raise AssertionError("WR CPU flash loader did not terminate")

    run_simulation(dut, [flash_model(), memory_model(), monitor()])
    return result, writes

# Flash Boot Tests ---------------------------------------------------------------------------------

def test_flash_boot_copies_little_endian_words():
    payload        = bytes(range(1, 17))
    result, writes = simulate_boot(build_boot_image(payload))
    assert result == {"ready": 1, "error": WRCPUFlashBoot.ERROR_NONE}
    assert writes == [
        (0x00, 0x04030201, 0xf),
        (0x04, 0x08070605, 0xf),
        (0x08, 0x0c0b0a09, 0xf),
        (0x0c, 0x100f0e0d, 0xf),
    ]


def test_flash_boot_rejects_bad_crc():
    image = bytearray(build_boot_image(b"firmware"))
    image[-1] ^= 0x80
    result, writes = simulate_boot(image)
    assert result == {"ready": 0, "error": WRCPUFlashBoot.ERROR_CRC}
    assert len(writes) == 2


def test_flash_boot_after_startupe2_clock_handoff():
    # UG470: the first three USRCCLKO cycles after EOS do not reach CCLK.
    result, writes = simulate_boot(build_boot_image(bytes(range(16))), startup_clocks=3)
    assert result == {"ready": 1, "error": WRCPUFlashBoot.ERROR_NONE}
    assert len(writes) == 4


def test_flash_boot_waits_for_memory_acknowledgment():
    result, writes = simulate_boot(build_boot_image(bytes(range(16))), memory_latency=8)
    assert result == {"ready": 1, "error": WRCPUFlashBoot.ERROR_NONE}
    assert len(writes) == 4


def test_flash_boot_timeout_keeps_cpu_in_reset():
    result, writes = simulate_boot(build_boot_image(bytes(range(16))), memory_latency=100)
    assert result == {"ready": 0, "error": WRCPUFlashBoot.ERROR_WB_TIMEOUT}
    assert writes == []
