#!/usr/bin/env python3
"""Fit per-board master trims from reciprocal Acorn/SPEC main-clock sweeps."""

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import linear_regression, mean


def fit_trims(rows, spec_linear_max, spec_plateau_min):
    # Each sweep holds the peer at 32768. Normalize by RX, then anchor Acorn's
    # sweep to its own measured midpoint. This avoids combining the offsets of
    # sequential reciprocal measurements as though they were simultaneous.
    points = {}
    for board in ('acorn', 'spec'):
        points[board] = []
        for row in rows:
            if row['board'] != board:
                continue
            code = int(row['code'])
            ref, rx = float(row['ref_hz']), float(row['rx_hz'])
            if (not 0 <= code <= 65535 or not ref > 0 or not rx > 0
                    or not math.isfinite(ref) or not math.isfinite(rx)):
                raise ValueError('Invalid DAC code or clock measurement')
            points[board].append((code, ref / rx))
        if len({x for x, _ in points[board]}) < 3:
            raise ValueError('Each board requires at least three distinct codes')
    anchors = [y for x, y in points['acorn'] if x == 32768]
    if not anchors:
        raise ValueError('Acorn sweep must include code 32768')
    anchor = mean(anchors)
    points['acorn'] = [(x, (y / anchor - 1) * 1e6) for x, y in points['acorn']]
    points['spec'] = [(x, (y - 1) * 1e6) for x, y in points['spec']]
    low = [y for x, y in points['spec'] if x == 0]
    high = [y for x, y in points['spec'] if x >= spec_plateau_min]
    if not low or not high:
        raise ValueError('SPEC sweep must include code 0 and its measured upper plateau')
    target = (mean(low) + mean(high)) / 2
    result = dict(reference='Acorn at 32768; relative frequency only',
        target_ppm=target, spec_range_ppm=[mean(low), mean(high)], boards={})
    for board, samples in points.items():
        linear = [(x, y) for x, y in samples if board == 'acorn' or x <= spec_linear_max]
        if len({x for x, _ in linear}) < 3:
            raise ValueError('Need at least three codes in each fitted linear region')
        fit = linear_regression([x for x, _ in linear], [y for _, y in linear])
        if fit.slope <= 0:
            raise ValueError('Expected a positive tuning slope')
        code = round((target - fit.intercept) / fit.slope)
        if not min(x for x, _ in linear) <= code <= max(x for x, _ in linear):
            raise ValueError('Centered trim lies outside the measured linear region')
        result['boards'][board] = dict(master_trim=code, ppm_per_code=fit.slope,
            max_fit_residual_ppm=max(abs(y - fit.slope * x - fit.intercept) for x, y in linear))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path, help='Columns: board,code,ref_hz,rx_hz')
    parser.add_argument('--spec-linear-max', type=int, required=True,
        help='Highest code in the measured SPEC linear region.')
    parser.add_argument('--spec-plateau-min', type=int, required=True,
        help='Lowest code in the measured SPEC upper plateau.')
    args = parser.parse_args()
    with args.csv.open() as stream:
        rows = list(csv.DictReader(stream))
    print(json.dumps(fit_trims(rows, args.spec_linear_max, args.spec_plateau_min), indent=2))


if __name__ == '__main__':
    main()
