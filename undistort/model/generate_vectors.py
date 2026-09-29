"""Generate binary32 arithmetic and Brown-map regression vectors (not RTL)."""
from pathlib import Path
import argparse
import struct
import numpy as np
from map_coord_fp32 import Camera, map_coordinate, fp32_bits

def decode(bits):
    return np.float32(struct.unpack('<f', struct.pack('<I', int(bits)))[0])

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    rng=np.random.default_rng(20260929)
    records=[]
    with np.errstate(all='ignore'):
        for op in range(5):
            for i in range(500):
                a=int(rng.integers(0,1<<32,dtype=np.uint64))
                b=int(rng.integers(0,1<<32,dtype=np.uint64))
                # Include normal, zero, and subnormal operands, but no Inf/NaN input.
                if (a>>23)&255==255:a^=1<<23
                if (b>>23)&255==255:b^=1<<23
                fa,fb=decode(a),decode(b)
                if op==0: value=np.float32(fa+fb)
                elif op==1: value=np.float32(fa-fb)
                elif op==2: value=np.float32(fa*fb)
                elif op==3: value=np.float32(fa/fb)
                else:value=np.float32(a)
                expected=fp32_bits(value)
                records.append(f'{op:01x}{a:08x}{b:08x}{expected:08x}')
    args.out.joinpath('fp_ops.mem').write_text('\n'.join(records)+'\n',encoding='ascii')
    points=[(0,0),(1279,0),(640,360),(0,719),(1279,719)]+[
        (int(rng.integers(0,1280)),int(rng.integers(0,720))) for _ in range(95)]
    camera=Camera()
    params=[fp32_bits(getattr(camera,k)) for k in ('fx','fy','cx','cy','k1','k2','k3','p1','p2')]
    args.out.joinpath('camera.mem').write_text('\n'.join(f'{p:08x}' for p in params)+'\n',encoding='ascii')
    rows=[]
    for x,y in points:
        sx,sy=map_coordinate(x,y,camera)
        rows.append(f'{x:04x}{y:04x}{fp32_bits(sx):08x}{fp32_bits(sy):08x}')
    args.out.joinpath('map_coords.mem').write_text('\n'.join(rows)+'\n',encoding='ascii')
    print(f'Generated {len(records)} FP32 and {len(rows)} map vectors in {args.out}')

if __name__=='__main__':main()
