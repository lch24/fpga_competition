"""Regenerate the LM fixture with the independent reference in this package.

The three stage lines keep the original physical-Brown truth and the original
initialiser convention (truth plus a small deterministic perturbation on each
active parameter); only the expectations now come from the Python solve in
generate_lm_fisheye.py, including the current lens-shape regularization.
Stages 0/1 remain diagnostic fixtures; production uses the full stage 2.
"""
import math
import struct
from pathlib import Path

from generate_lm_fisheye import (POINTS, STATE_N, VIEWS, WIDTH, HEIGHT, bits, optimize,
                                 pack_points, pack_state, predict, residual)

ROOT = Path(__file__).resolve().parents[3]
TRUTH = [math.log(800.0), math.log(820.0), .5, .5, .02, -.005, .001, -.002, 0.0]
for rv, t in [((.2, -.3, .1), (.3, -.2, 14.0)), ((-.3, .1, -.15), (-.2, .3, 16.0)),
              ((.1, .4, .2), (.1, .2, 18.0))]:
    TRUTH += list(rv) + [t[0], t[1], math.log(t[2])]


def stage_active(stage):
    active = [0, 1, 2, 3] + list(range(9, STATE_N))
    if stage > 0:
        active.append(4)
    if stage > 1:
        active += [5, 6, 7, 8]
    return active


def stage_ranges(active):
    full = (0, VIEWS * POINTS * 2)
    return [full if index < 9 else ((index - 9) // 6 * POINTS * 2, ((index - 9) // 6 + 1) * POINTS * 2)
            for index in active]


def build():
    observed = [(struct.unpack('<f', struct.pack('<f', u))[0],
                 struct.unpack('<f', struct.pack('<f', v))[0]) for u, v in predict(TRUTH)]
    lines, summary = [], []
    for stage in range(3):
        active = stage_active(stage)
        initial = TRUTH[:]
        for position, index in enumerate(active):
            initial[index] += (position % 5 - 2) * 1e-5
        result, accepted, converged = optimize(initial, observed, active, stage_ranges(active))
        _, cost = residual(result, observed)
        deviation = max(abs(result[i] - TRUTH[i]) / (1.0 + abs(TRUTH[i])) for i in range(STATE_N))
        assert converged and accepted >= 1, (stage, converged, accepted)
        # In this narrow-board case k2/k3 are weakly separated. Check the
        # observable full-field mapping, rather than demand identical individual
        # coefficients after adding the lens-shape prior.
        from check_lens_quality import mapping_error
        field_error=mapping_error(result,TRUTH,WIDTH,HEIGHT)
        assert max(field_error.values()) < .05, (stage,field_error,deviation)
        lines.append(f'{stage} 1 {accepted} {bits(cost):016x} {pack_state(initial)} '
                     f'{pack_points(observed)} {pack_state(result)}')
        summary.append(f'stage{stage} physical Brown model, active={len(active)} accepted={accepted} '
                       f'converged={int(converged)} cost={cost:.17g} '
                       f'max_relative_error_vs_truth={deviation:.3g}')
        print(f'LM_REFERENCE {summary[-1]}')
    (ROOT / 'data/calibration/lm_controller_vectors.txt').write_text('\n'.join(lines) + '\n')
    (ROOT / 'data/calibration/lm_controller_cases.txt').write_text('\n'.join(summary) + '\n')


if __name__ == '__main__':
    build()
