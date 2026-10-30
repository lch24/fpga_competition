"""Fisheye LM fixture: independent projection model plus independent LM solve.

The corners are synthesized from a known camera/pose state through a plain
textbook Brown model; the expectation comes from a damped Gauss-Newton solve
written here. Only the LM tuning constants (difference step, damping schedule,
termination thresholds) are taken from the documented program, because those are
the behaviour under test; the projection, the linear algebra and the whole
bookkeeping are separate code. Distortion starts at zero, which is the run-time
risk this fixture targets. The module is also the reference used to regenerate
`lm_controller_vectors.txt` (see generate_lm_reference.py).
"""
import math
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WIDTH, HEIGHT = 640, 480
ROWS, COLS, VIEWS = 5, 8, 3
POINTS = ROWS * COLS
STATE_N = 9 + 6 * VIEWS


def _f64(bits):
    """Decode the bit patterns build_lm.py uses for its LM constants."""
    return struct.unpack('<d', struct.pack('<Q', bits))[0]


DIFFERENCE_STEP = _f64(0x3eb0c6f7a0b5ed8d)
SCALE_FLOOR = _f64(0x3d719799812dea11)
COST_STOP = _f64(0x3c9cd2b297d889bc)
GRADIENT_STOP = _f64(0x3e45798ee2308c3a)
REDUCTION_STOP = _f64(0x3eb0c6f7a0b5ed8d)
STEP_STOP = _f64(0x3e112e0be826d695)
RELAXED_GRADIENT = _f64(0x3ee4f8b588e368f1)
# Initial pinhole fit cost above which the staged warm-ups are worth running.
STAGE_COST = _f64(0x4090e00000000000)

# fisheye-scale radial terms; k3 is the term the solver used to leave at zero
FISHEYE = dict(fx=430.0, fy=436.0, cx=0.5, cy=0.5, k1=-0.33, k2=0.19, k3=-0.11,
               p1=0.0012, p2=-0.0009)
POSES = [((0.18, -0.28, 0.09), (0.25, -0.18, 4.5)),
         ((-0.26, 0.12, -0.14), (-0.22, 0.30, 5.0)),
         ((0.09, 0.34, 0.20), (0.10, 0.22, 4.2))]


def bits(x):
    return struct.unpack('<Q', struct.pack('<d', x))[0]


def bits32(x):
    return struct.unpack('<I', struct.pack('<f', x))[0]


def board_xy(index):
    return (2 * (index % COLS) - (COLS - 1)) / 2.0, (2 * (index // COLS) - (ROWS - 1)) / 2.0


def rodrigues(r):
    a = math.sqrt(sum(x * x for x in r))
    if a < 1e-14:
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    K = [0.0, -r[2], r[1], r[2], 0.0, -r[0], -r[1], r[0], 0.0]
    s, c = math.sin(a), math.cos(a)
    return [(1.0 if i == j else 0.0) + s / a * K[3 * i + j]
            + (1.0 - c) / (a * a) * sum(K[3 * i + p] * K[3 * p + j] for p in range(3))
            for i in range(3) for j in range(3)]


def truth_state():
    state = [math.log(FISHEYE['fx']), math.log(FISHEYE['fy']), FISHEYE['cx'], FISHEYE['cy'],
             FISHEYE['k1'], FISHEYE['k2'], FISHEYE['p1'], FISHEYE['p2'], FISHEYE['k3']]
    for rv, t in POSES:
        state += list(rv) + [t[0], t[1], math.log(t[2])]
    return state


def predict(state):
    """u,v for every view and point, in view-major then point order."""
    k = [math.exp(state[0]), math.exp(state[1]), state[2] * WIDTH, state[3] * HEIGHT]
    dist = state[4:9]
    out = []
    for view in range(VIEWS):
        r = rodrigues(state[9 + 6 * view:12 + 6 * view])
        t = (state[12 + 6 * view], state[13 + 6 * view], math.exp(state[14 + 6 * view]))
        for index in range(POINTS):
            x, y = board_xy(index)
            xc = r[0] * x + r[1] * y + t[0]
            yc = r[3] * x + r[4] * y + t[1]
            zc = r[6] * x + r[7] * y + t[2]
            xn, yn = xc / zc, yc / zc
            r2 = xn * xn + yn * yn
            radial = 1.0 + dist[0] * r2 + dist[1] * r2 * r2 + dist[4] * r2 * r2 * r2
            xd = xn * radial + 2.0 * dist[2] * xn * yn + dist[3] * (r2 + 2.0 * xn * xn)
            yd = yn * radial + dist[2] * (r2 + 2.0 * yn * yn) + 2.0 * dist[3] * xn * yn
            out.append((k[0] * xd + k[2], k[1] * yd + k[3]))
    return out


def residual(state, observed):
    flat = []
    cost = 0.0
    for (u, v), (ou, ov) in zip(predict(state), observed):
        du, dv = u - ou, v - ov
        flat += [du, dv]
        cost += du * du + dv * dv
    return flat, cost


def solve_normal(a, b):
    """Gaussian elimination with partial pivoting; None when singular."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-300:
            return None
        m[col], m[pivot] = m[pivot], m[col]
        for row in range(col + 1, n):
            factor = m[row][col] / m[col][col]
            if factor:
                for j in range(col, n + 1):
                    m[row][j] -= factor * m[col][j]
    x = [0.0] * n
    for row in range(n - 1, -1, -1):
        x[row] = (m[row][n] - sum(m[row][j] * x[j] for j in range(row + 1, n))) / m[row][row]
    return x


def jacobian_columns(state, observed, active, ranges, base):
    """Scaled forward-difference columns; raises when a column is not range local."""
    columns, scales = [], []
    for position, index in enumerate(active):
        first, last = ranges[position]
        step = DIFFERENCE_STEP * (1.0 + abs(state[index]))
        plus = state[:]
        plus[index] += step
        rp, _ = residual(plus, observed)
        values = [0.0] * len(rp)
        for r in range(first, last):
            values[r] = (rp[r] - base[r]) / step
        for r in list(range(0, first)) + list(range(last, len(rp))):
            if values[r] != 0.0:
                raise ValueError(f'column {index} is not range local')
        norm = math.sqrt(sum(v * v for v in values))
        if norm < SCALE_FLOOR:
            norm = SCALE_FLOOR
        scale = 1.0 / norm
        columns.append([v * scale for v in values])
        scales.append(scale)
    return columns, scales


def optimize(state, observed, active, ranges=None, iterations=150, tolerance=COST_STOP,
             gradient_stop=GRADIENT_STOP, reduction_stop=REDUCTION_STOP, step_stop=STEP_STOP,
             max_tries=16, relaxed_gradient=RELAXED_GRADIENT):
    """Damped Gauss-Newton in scaled columns; result state and accepted steps.

    The policy follows the documented LM program: forward differences with a
    relative step against the stored baseline, columns normalised to unit norm
    (floored), Marquardt damping on the unit diagonal, fixed accept/reject
    schedule and the documented termination tests. The arithmetic and the whole
    projection model are new code, which is what makes agreement informative.
    """
    state = state[:]
    residual_vector, cost = residual(state, observed)
    damping = 1e-3
    accepted = 0
    if ranges is None:
        ranges = [(0, len(residual_vector))] * len(active)

    for _ in range(iterations):
        if cost < tolerance:
            break
        jacobian, scales = jacobian_columns(state, observed, active, ranges, residual_vector)
        n = len(active)
        normal = [[sum(jacobian[i][r] * jacobian[j][r] for r in range(len(residual_vector)))
                   for j in range(n)] for i in range(n)]
        gradient = [sum(jacobian[i][r] * residual_vector[r] for r in range(len(residual_vector)))
                    for i in range(n)]
        if max(abs(g) for g in gradient) < gradient_stop * (1.0 + math.sqrt(cost)):
            return state, accepted, True
        attempt = 0
        while True:
            damped = [[normal[i][j] + (damping if i == j else 0.0) for j in range(n)]
                      for i in range(n)]
            delta = solve_normal(damped, [-g for g in gradient])
            step = None
            if delta is not None:
                step = [delta[j] * scales[j] for j in range(n)]
                trial = state[:]
                for j, index in enumerate(active):
                    trial[index] += step[j]
                try:
                    _, trial_cost = residual(trial, observed)
                except (OverflowError, ValueError):
                    trial_cost = float('inf')
            else:
                trial, trial_cost = None, float('inf')
            reduction = cost - trial_cost
            if trial_cost < cost:
                state, cost = trial, trial_cost
                residual_vector, _ = residual(state, observed)
                damping = max(damping * 0.3, SCALE_FLOOR)
                accepted += 1
                stepnorm = max(abs(step[j]) / (1.0 + abs(state[active[j]] - step[j])) for j in range(n))
                if stepnorm < step_stop or reduction < reduction_stop * cost:
                    return state, accepted, True
                break
            attempt += 1
            damping *= 10.0
            if attempt >= max_tries:
                gmax = max(abs(g) for g in gradient)
                if gmax < relaxed_gradient * (1.0 + math.sqrt(cost)):
                    return state, accepted, True
                return state, accepted, False
    return state, accepted, cost < tolerance


def ranges_for(active):
    full = (0, VIEWS * POINTS * 2)
    return [full if index < 9 else ((index - 9) // 6 * POINTS * 2, ((index - 9) // 6 + 1) * POINTS * 2)
            for index in active]


def optimize_sequence(state, observed, sequences, **kwargs):
    """Mirror the documented staged policy for stage 2/3 requests.

    A good pinhole initial fit (cost below STAGE_COST) runs one phase with the
    requested columns. A large initial cost means strong distortion, so two
    warm-up phases (22 and 23 columns) run first. Each phase starts from the
    previous state with damping reset; accepted steps accumulate and only the
    final phase publishes convergence.
    """
    _, cost = residual(state, observed)
    if len(sequences) == 1 or len(sequences[-1]) < STATE_N - 1 or cost < STAGE_COST:
        active = sequences[-1]
        return optimize(state, observed, active, ranges_for(active), **kwargs)
    accepted_total = 0
    converged = True
    for active in sequences:
        state, accepted, converged = optimize(state, observed, active, ranges_for(active), **kwargs)
        accepted_total += accepted
        if not converged:
            break
    return state, accepted_total, converged


def pack_state(state):
    return ''.join(f'{bits(state[i]):016x}' for i in range(STATE_N - 1, -1, -1))


def pack_points(observed):
    parts = []
    for view in range(VIEWS - 1, -1, -1):
        for index in range(POINTS - 1, -1, -1):
            u, v = observed[view * POINTS + index]
            parts.append(f'{bits32(v):08x}{bits32(u):08x}')
    return ''.join(parts)


def build():
    truth = truth_state()
    observed = [(struct.unpack('<f', struct.pack('<f', u))[0], struct.unpack('<f', struct.pack('<f', v))[0])
                for u, v in predict(truth)]
    # The initializer supplies intrinsics from the image size, centred principal
    # point, poses from the homographies and zero distortion.
    initial = truth[:]
    initial[0], initial[1] = math.log(WIDTH), math.log(WIDTH)
    initial[2], initial[3] = 0.5, 0.5
    for j in (4, 5, 6, 7, 8):
        initial[j] = 0.0
    for view in range(VIEWS):
        for axis in range(3):
            initial[9 + 6 * view + axis] *= 0.7
        initial[12 + 6 * view] += 0.2
        initial[13 + 6 * view] -= 0.15
        initial[14 + 6 * view] += 0.25
    staged = [[0, 1, 2, 3] + list(range(9, STATE_N)),
              [0, 1, 2, 3] + list(range(9, STATE_N)) + [4],
              [0, 1, 2, 3] + list(range(9, STATE_N)) + [4, 5, 6, 7, 8]]
    result, accepted, converged = optimize_sequence(initial, observed, staged)
    _, cost = residual(result, observed)
    # The same fit with k3 left out must be visibly worse, otherwise the fixture
    # would not discriminate the term this change enables.
    blind, _, _ = optimize(initial, observed, [j for j in staged[-1] if j != 8], iterations=300)
    _, cost_without_k3 = residual(blind, observed)
    deviation = max(abs(result[i] - truth[i]) / (1.0 + abs(truth[i])) for i in range(STATE_N))
    radii = max(math.hypot((u - WIDTH / 2) / FISHEYE['fx'], (v - HEIGHT / 2) / FISHEYE['fy'])
                for u, v in observed)
    out = ROOT / 'data/calibration/lm_fisheye_vectors.txt'
    out.write_text(f'{2} 1 0 {bits(cost):016x} {pack_state(initial)} {pack_points(observed)} '
                   f'{pack_state(result)}\n')
    (ROOT / 'data/calibration/lm_fisheye_cases.txt').write_text(
        f'fisheye stage=2 accepted={accepted} cost={cost:.17g} cost_without_k3={cost_without_k3:.17g} '
        f'max_relative_error_vs_truth={deviation:.3g} observed_radius={radii:.3f}\n')
    print(f'FISHEYE_REFERENCE accepted={accepted} cost={cost:.6g} cost_without_k3={cost_without_k3:.6g} '
          f'deviation_vs_truth={deviation:.3g} observed_radius={radii:.3f}')
    assert cost < 1e-6, f'synthetic data did not reach the float32 noise floor ({cost})'
    assert cost_without_k3 > 20.0 * cost, f'fixture does not discriminate k3 ({cost_without_k3})'
    assert deviation < 2e-3, f'reference solve drifted from truth ({deviation})'
    return result, truth


if __name__ == '__main__':
    build()
