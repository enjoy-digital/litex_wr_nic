import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('wr_trim', Path(__file__).resolve().parents[1] / 'tools/wr_trim.py')
trim = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trim)


def sweep():
    # Synthetic oscillators: Acorn midpoint is 26 ppm low; SPEC clips at
    # code 40000. Each row's RX count includes a deliberate gate-scale error.
    rows = []
    for board in ('acorn', 'spec'):
        for code in (0, 8192, 16384, 32768, 38500, 40960, 65535):
            ratio = ((1 + (code - 32768) * 0.0045e-6) / (1 + 26e-6)
                if board == 'acorn' else 1 + (2 + min(code, 40000) * 0.00075) * 1e-6)
            rx = 62500000 * 1.00003
            rows.append(dict(board=board, code=code, ref_hz=rx * ratio, rx_hz=rx))
    return rows


def test_centers_usable_range_and_cancels_gate_scale():
    result = trim.fit_trims(sweep())
    assert result['target_ppm'] == pytest.approx(17)
    assert result['boards']['spec']['master_trim'] == 20000
    assert result['boards']['acorn']['master_trim'] == 36546
    assert result['boards']['acorn']['ppm_per_code'] == pytest.approx(0.0045)


def test_rejects_missing_anchor():
    with pytest.raises(ValueError, match='32768'):
        trim.fit_trims([r for r in sweep() if r['code'] != 32768])


def test_rejects_reversed_control():
    rows = sweep()
    for row in rows:
        if row['board'] == 'spec':
            row['ref_hz'] = 2 * row['rx_hz'] - row['ref_hz']
    with pytest.raises(ValueError, match='positive'):
        trim.fit_trims(rows)
