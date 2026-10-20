"""Directed coverage microcode/host-audit checks, without synthesis or a board."""
import copy
import json
import math
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts/calibration/engine'))
from assemble import assemble
from build_lm import Program, M, C, CONSTANTS
from build_init import coverage_setup, coverage_point, coverage_select, build
from check_init_program import run as initialize
from reference import Machine, bits
from inspect_calibration import coverage, audit, overlay


def f32(x):
    return struct.unpack('<f', struct.pack('<f', x))[0]


def packed(points):
    word = lambda x: struct.unpack('<I', struct.pack('<f', x))[0]
    return [word(x) | word(y) << 32 for x, y in points]


def microcode():
    # Execute the exact coverage instruction sequences embedded in build_init,
    # independently of DLT/pose. The separate real-input test below covers the
    # complete initializer so workspace lifetimes are not assumed by this test.
    p = Program()
    p.emit('MOVI A7,0')
    coverage_setup(p)
    p.seti('t', 0); p.icopy('pa', 'init_raw')
    p.label('point')
    p.il(0, 'pa')
    p.emit('LD F0,A0,0', 'FGET32 F1,F0', 'FGET32.1 F2,F0', 'FCVT F1,F1', 'FCVT F2,F2')
    p.st('h8', 1); p.st('h9', 2)
    coverage_point(p)
    p.addi('pa', 'pa', 1); p.addi('t', 't', 1); p.ib('t', 3, 'points', 'point')
    coverage_select(p); p.emit('END 0')
    return assemble('\n'.join(p.lines), 512)[0]


def main():
    rom = microcode()
    tested = 0
    for width, height in ((1280, 720), (640, 480), (1281, 721)):
        x0, x1, y0, y1 = (width-1)/4, 3*(width-1)/4, (height-1)/4, 3*(height-1)/4
        center = (width/2, height/2)
        bands = [(x0, center[1]), (x1, center[1]), (center[0], y0), (center[0], y1)]
        cases = [[center]*120]
        cases += [sum(([p]*n for p in bands), []) for n in (1, 3, 4, 5)]
        cases += [sum(([p]*(3 if i == missing else 4) for i, p in enumerate(bands)), []) for missing in range(4)]
        # Values immediately inside/outside the threshold and native FP32 rounding.
        cases += [[(x0+.25, y0+.25), (x1-.25, y1-.25)]*8,
                  [(x0-.25, y0-.25), (x1+.25, y1+.25)]*8]
        for points in cases:
            points = [(f32(x), f32(y)) for x, y in points]
            m = Machine(rom, 2048, 2048)
            m.mem.update({i: 0 for i in range(288)})
            m.mem.update({C[k]: v for k, v in CONSTANTS.items()})
            for key, value in dict(fwidth=bits(width), fheight=bits(height), init_raw=1024,
                                   points=len(points)).items():
                m.mem[M[key]] = value
            for i, value in enumerate(packed(points)):
                m.mem[1024+i] = value
            assert m.run(0, len(rom)) == 0
            expected = coverage(points, width, height)
            assert bool(m.mem[M['fit_k3']]) == expected['fit_k3']
            assert {edge: m.mem[M['coverage_'+edge]] for edge in ('left', 'right', 'top', 'bottom')} == expected['edge_counts']
            tested += 1
    raw = json.loads((ROOT/'data/calibration/uart_fold_corners.json').read_text())
    report = audit(raw)
    assert report['corner_geometry_ok'] and not report['fit_k3']
    assert report['edge_counts'] == dict(left=0, right=4, top=0, bottom=4)
    initial_rom, _ = build()
    code, _, _, policy = initialize(initial_rom, 3, 5, 8, 1280, 720,
                                    packed([e['xy'] for e in raw['corners']]), return_policy=True)
    assert code == 0 and policy['edge_counts'] == report['edge_counts'] and not policy['fit_k3']
    # Complete initializer on wide-coverage, independently projected pinhole views.
    wide = []
    for angle, tz in ((.15, 9.5), (-.18, 9.8), (.2, 9.4)):
        for row in range(5):
            for col in range(8):
                x, y = col-3.5, row-2
                z = math.sin(angle)*y+tz
                wide.append((f32(640*x/z+319.5), f32(640*math.cos(angle)*y/z+239.5)))
    expected = coverage(wide, 640, 480)
    assert expected['fit_k3']
    code, _, _, policy = initialize(initial_rom, 3, 5, 8, 640, 480, packed(wide), return_policy=True)
    assert code == 0 and policy == dict(fit_k3=True, edge_counts=expected['edge_counts'])
    bad = copy.deepcopy(raw)
    bad['corners'][3]['xy'] = bad['corners'][12]['xy']
    assert not audit(bad)['corner_geometry_ok']
    for xy in ((-1, 100), (1280, 100), (float('nan'), 100)):
        bad = copy.deepcopy(raw); bad['corners'][0]['xy'] = xy
        assert not audit(bad)['corner_geometry_ok']
    bad = copy.deepcopy(raw); bad['corners'][0]['index'] = 8
    try:
        audit(bad)
    except ValueError:
        pass
    else:
        raise AssertionError('Non-contiguous indices accepted')
    ET.fromstring(overlay([e['xy'] for e in raw['corners'][:40]], 1280, 720, 5, 8))
    print(f'PASS calibration quality: {tested} microcode boundary cases, 2 full initializers, malformed corners, SVG')


if __name__ == '__main__':
    main()
