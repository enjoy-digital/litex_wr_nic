"""Check the fitted model against a closed-form discrete one-pole loop."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from wr_loop_analysis import margins, model_response


def test_proportional_integrating_plant_matches_closed_form():
    gain, kp, rate = 1.2, .15, 3814.697265625
    product = gain * kp
    expected = -(1-product)**np.arange(100)
    np.testing.assert_allclose(model_response(100, gain, 0, kp, 0), expected)
    result = margins(gain, 0, kp, 0, rate)
    angle = 2*np.arcsin(product/2)
    assert result['crossover_hz'] == pytest.approx(angle*rate/(2*np.pi), rel=2e-4)
    assert result['phase_margin_deg'] == pytest.approx(90-angle*90/np.pi, abs=.01)
    assert result['gain_margin_db'] == pytest.approx(20*np.log10(2/product), abs=.01)
