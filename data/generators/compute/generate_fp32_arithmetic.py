"""Exact rational binary32 oracle: one final nearest-even rounding, no NumPy."""
from pathlib import Path
from fractions import Fraction
import random
import argparse
def generate(out):
    out.mkdir(parents=True,exist_ok=True)
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
    (out / 'fp_ops.mem').write_text('\n'.join(fp32_rows) + '\n')

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True)
    generate(parser.parse_args().out)
