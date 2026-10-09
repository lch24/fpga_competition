"""Independent integer references; run from any directory, MODELSIM_BIN optional."""
import math
import os
from pathlib import Path
import random
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / 'build/image/integer_detection'
BUILD.mkdir(parents=True, exist_ok=True)
SIM = Path(os.environ.get('MODELSIM_BIN', r'E:\pangu\Modelsim10.1c\win64'))
def bits(x):
    return struct.unpack('<I', struct.pack('<f', x))[0]
rng = random.Random(761)
with (BUILD / 'harris.txt').open('w') as f:
    tensors = [(0, 0, 0), (9363600, 9363600, 9363600),
               (9363600, -9363600, 9363600), (9363600, 0, 9363600)]
    for _ in range(2000):
        gradients = [(rng.randint(-1020, 1020), rng.randint(-1020, 1020)) for _ in range(9)]
        tensors.append((sum(x*x for x,y in gradients), sum(x*y for x,y in gradients), sum(y*y for x,y in gradients)))
    for a,b,c in tensors:
        score = max(0, 25*(a*c-b*b)-(a+c)**2)
        f.write(f'{a:08x} {b & 0xffffffff:08x} {c:08x} {bits(score):08x}\n')

def lround(x):
    return int(math.copysign(math.floor(abs(x)+.5), x))
with (BUILD / 'rings.txt').open('w') as f:
    for scene in range(180):
        # Chess quadrants, edges, low contrast, noise and off-center samples.
        cx,cy = 31.5 + rng.uniform(-2,2), 31.5 + rng.uniform(-2,2)
        if scene % 12 == 0:
            cx = 2.5
        image = []
        for y in range(64):
            for x in range(64):
                if scene%4==0: value = rng.randrange(256)
                elif scene%4==1: value = 60+((x>=32)^(y>=32))*140
                elif scene%4==2: value = 100+((x>=32)^(y>=32))*10
                else: value = 30+(x>=32)*180
                image.append(value)
        for radius in (4,6,8):
            x,y=lround(cx),lround(cy)
            passed=False
            if radius+1<=x<64-radius-1 and radius+1<=y<64-radius-1:
                values=[image[(y+lround(radius*math.sin(k*math.pi/16)))*64+x+lround(radius*math.cos(k*math.pi/16))] for k in range(32)]
                sm=[values[(k-1)%32]+2*values[k]+values[(k+1)%32] for k in range(32)]
                lo,hi=min(sm),max(sm)
                tr=[k for k in range(32) if (2*sm[k]>hi+lo)!=(2*sm[(k-1)%32]>hi+lo)]
                opposite=sum(abs(sm[k]-sm[(k+16)%32]) for k in range(32))
                passed=hi-lo>=80 and len(tr)==4 and 25*opposite<=224*(hi-lo) and all(3<=(tr[(k+1)%4]-tr[k])%32<=13 for k in range(4))
            f.write(f'{bits(cx):08x} {bits(cy):08x} {bits(radius):08x} {int(passed)}\n')
            f.write(' '.join(f'{v:02x}' for v in image)+'\n')

def run(tool, *args):
    result=subprocess.run([str(SIM/tool),*args],cwd=BUILD,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    (BUILD/(tool+'.log')).write_text(result.stdout,encoding='utf-8')
    if result.returncode or '** Error:' in result.stdout or '** Fatal:' in result.stdout:
        raise RuntimeError(result.stdout[-6000:])
    return result.stdout
if not (BUILD/'work').exists(): run('vlib.exe','work')
run('vlog.exe','-sv','+incdir+'+str(ROOT/'rtl/include'),
    str(ROOT/'rtl/image/kernels/harris_response.v'),str(ROOT/'rtl/image/features/ring_check.sv'),
    str(ROOT/'tb/image/tb_integer_detection.sv'))
(BUILD/'run.do').write_text('''onfinish stop
onerror {quit -code 1 -f}
onbreak {
 if {[examine -radix unsigned /tb_integer_detection/done] != 1} {quit -code 1 -f}
 echo "PASS integer detection tensors=[examine -radix unsigned /tb_integer_detection/n] rings=[examine -radix unsigned /tb_integer_detection/rings]"
 quit -code 0 -f
}
run -all
quit -code 1 -f
''')
out=run('vsim.exe','-c','-voptargs=+acc','work.tb_integer_detection','-do','run.do')
if 'PASS integer detection' not in out: raise RuntimeError(out[-2000:])
print(out[out.index('# PASS'):].splitlines()[0])
