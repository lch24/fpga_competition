"""Independent Q20 bilinear oracle, numerical error bound and RTL stall checks."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'common'))
from tools import modelsim_bin

import json, random, struct, subprocess

ROOT=Path(__file__).resolve().parents[2]
BUILD=ROOT/'build/system/bilinear_fixed'
MS=modelsim_bin()
def bits(x): return struct.unpack('>I',struct.pack('>f',x))[0]
def value(x): return struct.unpack('>f',struct.pack('>I',x))[0]
def f(x): return value(bits(x))
def main():
    BUILD.mkdir(parents=True,exist_ok=True)
    rng=random.Random(61006)
    edges=[0,1,0x007fffff,0x00800000,bits(2**-21),bits(2**-20),bits(.5),0x3f7fffff,bits(1)]
    cases=[]
    for dx in edges:
        for dy in edges:
            for pixels in [(0,255,255,0),(255,0,0,255),(255,255,255,255),(0,0,0,0)]:
                cases.append((*pixels,dx,dy))
    for _ in range(5000):
        cases.append((*[rng.randrange(256) for _ in range(4)],bits(rng.random()),bits(rng.random())))
    rows=[];max_exact_error=0.;max_float_error=0.
    for a,b,c,d,dx,dy in cases:
        x,y=value(dx),value(dy);qx,qy=int(x*2**20),int(y*2**20)
        top=a*2**20+(b-a)*qx;bot=c*2**20+(d-c)*qx
        fixed=top*2**20+(bot-top)*qy
        expected=bits(fixed/2**40)
        exact=(1-y)*((1-x)*a+x*b)+y*((1-x)*c+x*d)
        old=f(f(f(1-y)*f(f(f(1-x)*a)+f(x*b)))+f(y*f(f(f(1-x)*c)+f(x*d))))
        max_exact_error=max(max_exact_error,abs(value(expected)-exact))
        max_float_error=max(max_float_error,abs(value(expected)-old))
        assert abs(value(expected)-exact)<=510/2**20+2**-17+1e-12
        rows.append(f'{a:02x}{b:02x}{c:02x}{d:02x}{dx:08x}{dy:08x}{expected:08x}')
    (BUILD/'vectors.hex').write_text('\n'.join(rows)+'\n')
    result=dict(vectors=len(rows),max_error_gray=max_exact_error,max_error_vs_fp32_gray=max_float_error,
                analytic_bound_gray=510/2**20+2**-17)
    def run(tool,*args):
        p=subprocess.run([str(MS/(tool+'.exe')),*args],cwd=BUILD,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        (BUILD/(tool+'.log')).write_text(p.stdout)
        if p.returncode:raise RuntimeError(p.stdout[-5000:])
        return p.stdout
    if not (BUILD/'work').exists():run('vlib','work')
    run('vlog','-sv',str(ROOT/'rtl/image/kernels/bilinear_core.v'),str(ROOT/'tb/compute/tb_bilinear_fixed.sv'))
    (BUILD/'run.do').write_text('onfinish stop\nonerror {quit -code 1 -f}\nonbreak {\n'
        f'if {{[examine /tb_bilinear_fixed/finished]!=1 || [examine -radix unsigned /tb_bilinear_fixed/checked]!={len(rows)}}} {{quit -code 1 -f}}\n'
        'echo "PASS bilinear fixed"\nquit -code 0 -f\n}\nrun -all\n')
    output=run('vsim','-c','-voptargs=+acc=rn',f'-gCOUNT={len(rows)}','work.tb_bilinear_fixed','-do','run.do')
    assert 'PASS bilinear fixed' in output and '** Fatal:' not in output,output[-5000:]
    (BUILD/'result.json').write_text(json.dumps(result,indent=2))
    print('PASS',json.dumps(result))
if __name__=='__main__':main()
