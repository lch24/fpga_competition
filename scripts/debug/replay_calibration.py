"""Replay UART corners with the independent LM reference and audit map folding.

No artificial-distortion preset is supplied to either fit. Input is an export
from uart_monitor.py --corners; dimensions must describe the captured images.
"""
import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/calibration/engine'))
import generate_lm_fisheye as model
from check_init_program import run as initialize
from build_init import build


def camera(state, width, height):
    return dict(zip(('fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'p1', 'p2', 'k3'),
                    [math.exp(state[0]), math.exp(state[1]), state[2]*width,
                     state[3]*height] + state[4:9]))


def audit(k, width, height):
    failures = []
    minimum = (float('inf'), None)
    for iy in range(25):
        for ix in range(33):
            u, v = (width-1)*ix/32, (height-1)*iy/24
            x, y = (u-k['cx'])/k['fx'], (v-k['cy'])/k['fy']
            r2 = x*x+y*y
            radial = 1+k['k1']*r2+k['k2']*r2*r2+k['k3']*r2**3
            dr = k['k1']+2*k['k2']*r2+3*k['k3']*r2*r2
            a = radial+2*x*x*dr+2*k['p1']*y+6*k['p2']*x
            b = 2*x*y*dr+2*k['p1']*x+2*k['p2']*y
            d = radial+2*y*y*dr+6*k['p1']*y+2*k['p2']*x
            det = a*d-b*b
            if det < minimum[0]:
                minimum = (det, [u, v])
            if not math.isfinite(det) or a <= 0 or d <= 0 or det <= 1e-4:
                failures.append(dict(pixel=[u, v], a=a, d=d, determinant=det))
    return dict(failed_samples=len(failures), total_samples=825,
                minimum_determinant=minimum[0], minimum_at=minimum[1],
                first_failure=failures[0] if failures else None)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('corners', type=Path)
    p.add_argument('--width', type=int, default=1280)
    p.add_argument('--height', type=int, default=720)
    p.add_argument('--rows', type=int, default=5)
    p.add_argument('--cols', type=int, default=8)
    p.add_argument('--out', type=Path, default=ROOT/'build/debug/calibration_replay.json')
    args = p.parse_args()
    data = json.loads(args.corners.read_text())
    entries = data['corners']
    assert [e['index'] for e in entries] == list(range(len(entries)))
    observed = [e['xy'] for e in entries]
    count = args.rows*args.cols
    assert len(observed) % count == 0
    views = len(observed)//count
    model.WIDTH, model.HEIGHT = args.width, args.height
    model.ROWS, model.COLS, model.VIEWS = args.rows, args.cols, views
    model.POINTS, model.STATE_N = count, 9+6*views
    packed = [model.bits32(x) | model.bits32(y) << 32 for x, y in observed]
    rom, _ = build()
    status, initial, _ = initialize(rom, views, args.rows, args.cols,
                                    args.width, args.height, packed)
    assert status == 0, status
    source = args.corners.resolve()
    try:
        source = source.relative_to(ROOT)
    except ValueError:
        pass
    report = dict(source=source.as_posix(), initial=initial, views=views,
                  width=args.width, height=args.height, rows=args.rows, cols=args.cols)
    board = data['status'].get('candidate_params') or data['status'].get('params')
    if board:
        report['board'] = dict(camera=board, rms=data['status'].get('rms'),
                               map=audit(board, args.width, args.height))
        print('BOARD', json.dumps(report['board']), flush=True)
    for name, distortion in [('full', [4,5,6,7,8]), ('fixed_k3', [4,5,6,7])]:
        active = [0,1,2,3] + list(range(9, model.STATE_N)) + distortion
        ranges = [(0, len(observed)*2) if i < 9 else
                  ((i-9)//6*count*2, ((i-9)//6+1)*count*2) for i in active]
        result, accepted, converged = model.optimize(initial, observed, active, ranges)
        _, cost = model.residual(result, observed)
        k = camera(result, args.width, args.height)
        report[name] = dict(camera=k, rms=math.sqrt(cost/len(observed)),
                            state=result, accepted=accepted, converged=converged,
                            map=audit(k, args.width, args.height))
        print(name, json.dumps(report[name]), flush=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
