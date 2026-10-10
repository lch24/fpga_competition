"""Independent numerical and cache/service protocol checks for compact RTL."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'common'))
from tools import modelsim_bin

import subprocess
import struct
import random
import math
import sys
from fractions import Fraction

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / 'build/system/resource_units'
MODELSIM = modelsim_bin()

def run(executable, *arguments):
    result = subprocess.run([str(executable), *map(str, arguments)], cwd=BUILD,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (BUILD / (Path(executable).stem + '.log')).write_text(result.stdout, encoding='utf-8')
    if result.returncode:
        raise RuntimeError(result.stdout[-6000:])
    return result.stdout

def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    rng = random.Random(70521)
    def decode32(bits):
        exponent = (bits >> 23) & 255
        mantissa = (bits & 0x7fffff) | (0x800000 if exponent else 0)
        power = exponent - 150 if exponent else -149
        return (-1 if bits >> 31 else 1) * Fraction(mantissa) * Fraction(2) ** power
    def encode32(value, negative_zero=False):
        sign = 0x80000000 if value < 0 or (value == 0 and negative_zero) else 0
        value = abs(value)
        if not value:
            return sign
        exponent = value.numerator.bit_length() - value.denominator.bit_length()
        if value < Fraction(2) ** exponent:
            exponent -= 1
        scaled = value * Fraction(2) ** (23 - max(-126, exponent))
        rounded, remainder = divmod(scaled.numerator, scaled.denominator)
        if 2 * remainder > scaled.denominator or (2 * remainder == scaled.denominator and rounded & 1):
            rounded += 1
        if rounded >= (1 << 24):
            rounded >>= 1
            exponent += 1
        if exponent > 127:
            return sign | 0x7f800000
        field = max(1, exponent + 127) if rounded >= (1 << 23) else 0
        return sign | (field << 23) | (rounded & 0x7fffff)
    fp32_rows = []
    edges32 = [0, 1, 0x7fffff, 0x800000, 0x3f800000, 0x7f7fffff, 0x80000000]
    for op in range(5):
        for index in range(500):
            a, b = rng.getrandbits(32), rng.getrandbits(32)
            if index < len(edges32) ** 2:
                a, b = edges32[index // len(edges32)], edges32[index % len(edges32)]
            if (a >> 23) & 255 == 255: a ^= 1 << 23
            if (b >> 23) & 255 == 255: b ^= 1 << 23
            x, y = decode32(a), decode32(b)
            sign = bool((a ^ b) >> 31)
            if op == 3 and not y:
                result = 0x7fc00000 if not x else ((0x80000000 if sign else 0) | 0x7f800000)
            else:
                value = [lambda: x+y, lambda: x-y, lambda: x*y, lambda: x/y, lambda: Fraction(a)][op]()
                negative_zero = sign if op in (2, 3) else (op < 2 and x == y == 0 and bool(a >> 31) and bool((b >> 31) ^ op))
                result = encode32(value, negative_zero)
            fp32_rows.append(f'{op:x}{a:08x}{b:08x}{result:08x}')
    (BUILD / 'fp_ops.mem').write_text('\n'.join(fp32_rows) + '\n')
    special = [0, 1, 0x000fffffffffffff, 0x0010000000000000,
               0x3ff0000000000000, 0x4000000000000000, 0x7fefffffffffffff,
               0x7ff0000000000000, 0x7ff8000000000000]
    pairs = [(a | sign, b) for sign in (0, 1 << 63) for a in special for b in special]
    pairs += [(rng.getrandbits(64), rng.getrandbits(64)) for _ in range(5000)]
    rows = []
    for a, b in pairs:
        x, y = [struct.unpack('>d', struct.pack('>Q', bits))[0] for bits in (a, b)]
        sign = (a ^ b) & (1 << 63)
        if math.isnan(x) or math.isnan(y) or (x == 0 and y == 0) or (math.isinf(x) and math.isinf(y)):
            z = 0x7ff8000000000000
        elif y == 0:
            z = sign | 0x7ff0000000000000
        else:
            z = struct.unpack('>Q', struct.pack('>d', x / y))[0]
        rows.append(f'{a:016x} {b:016x} {z:016x}')
    (BUILD / 'division_vectors.txt').write_text('\n'.join(rows) + '\n')
    products=[]
    # Detector double operands originate from finite FP32/int values. Keep
    # the established normal-product domain, including both signed zeros.
    for i in range(5000):
        bits=[(rng.getrandbits(1)<<63)|(rng.randrange(850,1170)<<52)|rng.getrandbits(52) for _ in range(2)]
        if i<4: bits=[(i&1)<<63, ((i>>1)&1)<<63]
        a,b=bits
        x,y=[struct.unpack('>d',struct.pack('>Q',v))[0] for v in bits]
        z=struct.unpack('>Q',struct.pack('>d',x*y))[0]
        products.append(f'{a:016x} {b:016x} {z:016x}')
    (BUILD / 'multiply_vectors.txt').write_text('\n'.join(products)+'\n')
    if not (BUILD / 'work').exists():
        run(MODELSIM / 'vlib.exe', 'work')
    sources = []
    for directory in ('rtl',):
        sources += [p for p in (ROOT / directory).rglob('*') if p.suffix in ('.sv', '.v') and 'board' not in p.parts and p.name != 'calibrated_view_top.v']
    sources.sort(key=str)
    tests = {'tb_calib_geometry': ['checked==338'], 'tb_resource_math': ['checked==2500', f'div_checked=={len(rows)}', 'mul_checked==5000', 'reset_checked==3', 'mul_reset_checked==4'],
             'tb_ddr_recovery': ['checked==2'],
             'tb_gray_cache': ['checked==1153', 'faults==3'], 'tb_fp_pool': ['accepted==4', 'returned==3'],
             'tb_ddr_pyramid': ['checked==102400', 'native_passes==2']}
    if '--only' in sys.argv:
        selected = sys.argv[sys.argv.index('--only') + 1]
        if selected not in tests:
            raise ValueError('Unknown test: ' + selected)
        tests = {selected: tests[selected]}
    import shutil
    shutil.copytree(ROOT / 'data/rom', BUILD / 'data/rom', dirs_exist_ok=True)
    sources += [ROOT / 'tb/models/ddr_memory_model.sv']
    sources += [next((ROOT / 'tb').rglob(f'{test}.sv')) for test in tests]
    manifest = ['+incdir+' + str(ROOT / directory).replace('\\', '/') for directory in
                ('rtl/include', 'rtl/compute/float', 'rtl/include', 'tb/models')]
    manifest += ['"' + p.as_posix() + '"' for p in sources]
    (BUILD / 'sources.f').write_text('\n'.join(line if line.startswith(chr(34)) else chr(34)+line+chr(34) for line in manifest))
    run(MODELSIM / 'vlog.exe', '-sv', '-f', 'sources.f')
    for test, checks in tests.items():
        conditions = [f'[examine -radix unsigned /{test}/finished]!=1']
        conditions += [f'[examine -radix unsigned /{test}/{name}]!={value}'
                       for name, value in (check.split('==') for check in checks)]
        do = 'transcript on\nonfinish stop\nonerror {quit -code 1 -f}\nonbreak {\n'
        if test == 'tb_resource_math':
            do += ' echo "MATH i=[examine -radix unsigned /tb_resource_math/i] op=[examine /tb_resource_math/req_op] a=[examine -radix hex /tb_resource_math/req_a] b=[examine -radix hex /tb_resource_math/req_b] got=[examine -radix hex /tb_resource_math/rsp_result] expected=[examine -radix hex /tb_resource_math/expected32]"\n'
        do += ' if {' + ' || '.join(conditions) + '} {quit -code 1 -f}\n'
        do += f' echo "PASS {test}"\n quit -code 0 -f\n}}\nrun -all\n'
        (BUILD / 'run.do').write_text(do)
        output = run(MODELSIM / 'vsim.exe', '-c', f'-voptargs=+acc=rn+{test} -O5', 'work.' + test, '-do', 'run.do')
        (BUILD / (test + '.log')).write_text(output, encoding='utf-8')
        if 'PASS ' + test not in output or '** Fatal:' in output or '** Error:' in output:
            raise RuntimeError(output[-5000:])
        print('PASS', test, ', '.join(checks), flush=True)

if __name__ == '__main__':
    main()
