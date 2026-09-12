from pathlib import Path
import sys

import pytest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))
from wr_pll_trace import Decoder, MODULUS, Trace


def packet(source, sample, milliseconds, error=-17, dac=32768):
    fields = [(7, milliseconds), (5, sample), (0, dac), (1, error)]
    if source == 1:
        fields[:0] = [(8, -100), (9, 200), (2, 16384), (3, 16385)]
    words = [(source << 28) | (field << 24) | (value & (MODULUS - 1)) for field, value in fields]
    words[-1] |= 0x80000000
    return words


def test_chunk_boundaries_signed_error_and_independent_sources():
    decoder = Decoder(16)
    main = packet(1, 200, 1000)
    helper = packet(0, 300, 1001, error=31)
    assert not decoder.feed(main[:3])
    result = decoder.feed(main[3:] + helper)
    assert [(f['source'], f['error'], f['dac']) for f in result] == [(1, -17, 32768), (0, 31, 32768)]
    assert result[0]['phase_current'] == -100
    assert decoder.gaps == decoder.discarded == 0


def test_sample_and_timer_wrap_are_not_gaps():
    decoder = Decoder(16)
    decoder.feed(packet(1, MODULUS - 8, MODULUS - 2))
    result = decoder.feed(packet(1, 8, 2))[0]
    assert not result['gap']
    assert result['sample'] == 16
    assert result['elapsed_ms'] == 4


def test_fifo_loss_and_restart_are_visible():
    decoder = Decoder(16)
    decoder.feed(packet(1, 200, 1000))
    assert decoder.feed(packet(1, 232, 1008))[0]['gap']
    assert decoder.feed(packet(1, 0, 1009))[0]['gap']
    assert decoder.gaps == 2


def test_event_and_initial_partial_packet_do_not_create_samples():
    decoder = Decoder(16)
    assert decoder.feed(packet(1, 200, 1000)[-2:]) == []
    assert decoder.discarded == 1
    assert decoder.feed([0x96000004]) == [{'source': 1, 'event': 4}]
    assert decoder.samples == {}


def test_duplicate_fields_and_mixed_sources_are_rejected():
    decoder = Decoder(16)
    data = packet(1, 200, 1000)
    data[0] = data[1]
    assert decoder.feed(data) == []
    data = packet(1, 200, 1000)
    data[0] &= ~(7 << 28)
    assert decoder.feed(data) == []
    assert decoder.discarded == 2


def test_unframed_data_is_bounded():
    with pytest.raises(ValueError, match='end-of-sample'):
        Decoder(16).feed([0] * 33)


def test_native_burst_transitions_and_losses():
    decoder = Decoder(16)
    def sample(n, stride):
        return [(1 << 28) | (11 << 24) | stride] + packet(1, n, 1000)
    decoder.feed(sample(16, 16))
    assert not decoder.feed(sample(23, 1))[0]['gap']
    assert not decoder.feed(sample(24, 1))[0]['gap']
    assert decoder.feed(sample(26, 1))[0]['gap']
    assert not decoder.feed(sample(32, 16))[0]['gap']
    assert decoder.feed(sample(64, 16))[0]['gap']
    assert decoder.gaps == 2


def test_capture_summary_is_a_snapshot():
    trace = Trace.__new__(Trace)
    trace.decoder = Decoder(16)
    trace.high_water = 0
    trace.decoder.feed([(1 << 28) | (11 << 24) | 1] + packet(1, 16, 1000))
    initial = trace.summary()
    trace.decoder.feed([(1 << 28) | (11 << 24) | 1] + packet(1, 17, 1001))
    assert initial['native_samples'] == {1: 1}
    assert trace.summary()['native_samples'] == {1: 2}
