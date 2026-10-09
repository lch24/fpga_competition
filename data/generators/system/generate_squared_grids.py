"""Squared-grid reference in double precision, using FP32 coordinate differences."""
import math
from pathlib import Path
import random
import struct
ROOT = Path(__file__).resolve().parents[3]
def f32(x): return struct.unpack('<f', struct.pack('<f', x))[0]
def bits(x): return struct.unpack('<I', struct.pack('<f', x))[0]
def cost(g):
    total = 0.0
    sign = 0.0
    for row in range(5):
        for col in range(8):
            i=row*8+col
            triples=[]
            if col+2<8: triples.append((i,i+1,i+2,False))
            if row+2<5: triples.append((i,i+8,i+16,False))
            if col+1<8 and row+1<5:
                q=[i,i+1,i+9,i+8]
                triples.extend((q[k],q[(k+1)%4],q[(k+2)%4],True) for k in range(4))
            for a,b,d,cell in triples:
                x,y=(f32(g[b][k]-g[a][k]) for k in range(2))
                u,v=(f32(g[d][k]-g[b][k]) for k in range(2))
                d1,d2=x*x+y*y,u*u+v*v
                prod=d1*d2
                if cell:
                    cross=x*v-y*u
                    if prod<256 or cross*cross<.04*prod: return 1e30
                    if sign==0: sign=cross
                    if cross*sign<=0: return 1e30
                else:
                    dot=x*u+y*v
                    if d1<16 or d2<16 or d2<.3025*d1 or d2>3.24*d1 or dot<0 or dot*dot<.81*prod: return 1e30
                    total+=1-dot*dot/prod+4*(d2-d1)**2/(d1+d2)**2
    return total
rng=random.Random(5321)
target=ROOT/'data/fixtures/squared_grids.txt'
target.parent.mkdir(parents=True,exist_ok=True)
with target.open('w') as f:
    for case in range(32):
        angle=rng.uniform(-2,2)
        spacing=rng.uniform(8,36)
        points=[]
        for row in range(5):
            for col in range(8):
                x=col*spacing+rng.uniform(-.3,.3)
                y=row*spacing+rng.uniform(-.3,.3)
                points.append((f32(240+math.cos(angle)*x-math.sin(angle)*y),f32(240+math.sin(angle)*x+math.cos(angle)*y)))
        if case%4==0: points[11]=points[10]
        expected=cost(points)
        f.write(f'{struct.unpack("<Q",struct.pack("<d",expected))[0]:016x}\n')
        for x,y in points: f.write(f'{bits(x):08x} {bits(y):08x}\n')
print('Generated 32 independent squared-grid fixtures')
