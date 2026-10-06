"""Independent integer-isqrt oracle; normal inputs also checked against math.sqrt."""
import math
import random
import struct
from pathlib import Path


def bits(value):
    return struct.unpack('>Q', struct.pack('>d', value))[0]


def reference(x):
    if x >> 63 or x == 0:
        return 0
    exponent = ((x >> 52) & 2047) - 1023
    n = ((1 << 52) | (x & ((1 << 52) - 1))) << (56 + (exponent & 1))
    g = math.isqrt(n)
    mant = (g & ((1 << 54) - 1)) >> 2
    mant += bool(g & 2) and bool((g & 5) or g*g != n)
    return (((exponent // 2 + 1023 + (mant >> 52)) & 2047) << 52) | (mant & ((1 << 52)-1))


rng = random.Random(74103)
values = [0, 1 << 63, 1, (1 << 52)-1, 0x7ff0000000000000,
          0x7ff8000000000000, 0xfff0000000000000]
for e in range(1, 2047):
    values.extend((e << 52, (e << 52) | ((1 << 52)-1)))
values.extend((rng.randrange(1, 2047) << 52) | rng.getrandbits(52) for _ in range(3000))
values.extend(bits(float(i*i)) for i in range(1, 200))
values.extend(rng.getrandbits(64) for _ in range(200))
target = Path(__file__).resolve().parents[2] / 'system' / 'sqrt_vectors.txt'
target.parent.mkdir(exist_ok=True)
normal = 0
with target.open('w', encoding='ascii') as stream:
    for x in values:
        expected = reference(x)
        if not x >> 63 and 0 < ((x >> 52) & 2047) < 2047:
            host = bits(math.sqrt(struct.unpack('>d', struct.pack('>Q', x))[0]))
            assert expected == host, (hex(x), hex(expected), hex(host))
            normal += 1
        stream.write(f'{x:016x} {expected:016x}\n')
print(f'SQRT vectors={len(values)} host_sqrt_checked={normal}')
