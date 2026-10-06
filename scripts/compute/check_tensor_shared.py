"""FP64 tensor solver sharing: independent ordered arithmetic and CE/reset checks."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'common'))
from tools import modelsim_bin

import random,struct,subprocess,json
ROOT=Path(__file__).resolve().parents[2]
BUILD=ROOT/'build/system/tensor_shared'
MS=modelsim_bin()
def bits(x):return struct.unpack('>Q',struct.pack('>d',x))[0]
def main():
    BUILD.mkdir(parents=True,exist_ok=True);rng=random.Random(61007)
    cases=[(0.,0.,0.,0.,0.),(1.,1.,1.,1.,1.),(1e-10,0.,1e-10,0.,0.),(1.,0.,1.,2.,-3.)]
    for i in range(600):
        scale=2.**rng.randrange(-12,20)
        a=rng.uniform(.1,100)*scale;c=rng.uniform(.1,100)*scale
        b=rng.uniform(-.95,.95)*(a*c)**.5
        if i%11==0:b=(a*c)**.5
        if i%13==0:b=0.
        cases.append((a,b,c,rng.uniform(-10,10)*scale,rng.uniform(-10,10)*scale))
    rows=[];accepted=0
    for a,b,c,bx,by in cases:
        det=a*c-b*b;trace=a+c;ok=not(trace<1e-8 or det<=(1e-5*trace)*trace)
        dx=(c*bx-b*by)/det if ok else 0.;dy=(a*by-b*bx)/det if ok else 0.
        accepted+=ok
        rows.append(str(int(ok))+''.join(f'{bits(x):016x}' for x in (a,b,c,bx,by,dx,dy)))
    (BUILD/'vectors.hex').write_text('\n'.join(rows)+'\n')
    def run(tool,*args):
        p=subprocess.run([str(MS/(tool+'.exe')),*args],cwd=BUILD,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        (BUILD/(tool+'.log')).write_text(p.stdout)
        if p.returncode:raise RuntimeError(p.stdout[-5000:])
        return p.stdout
    if not (BUILD/'work').exists():run('vlib','work')
    sources=list((ROOT/'rtl/compute/float/stream').glob('*.v'))
    sources += list((ROOT/'rtl/common').glob('*.v'))
    sources += [ROOT/'rtl/compute/float/fp_divsqrt.v',ROOT/'rtl/image/kernels/tensor_solve.v',ROOT/'tb/compute/tb_tensor_shared.sv']
    run('vlog','-sv','+incdir+'+str(ROOT/'rtl/compute/float'),'+incdir+'+str(ROOT/'rtl/include'),*map(str,sources))
    (BUILD/'run.do').write_text('onfinish stop\nonerror {quit -code 1 -f}\nonbreak {\n'
        f'if {{[examine /tb_tensor_shared/finished]!=1 || [examine -radix unsigned /tb_tensor_shared/checked]!={len(rows)}}} {{quit -code 1 -f}}\n'
        'echo "PASS tensor shared cycles=[examine -radix unsigned /tb_tensor_shared/total_cycles]"\nquit -code 0 -f\n}\nrun -all\n')
    output=run('vsim','-c','-voptargs=+acc=rn',f'-gCOUNT={len(rows)}','work.tb_tensor_shared','-do','run.do')
    assert 'PASS tensor shared cycles=' in output and '** Fatal:' not in output,output[-5000:]
    report=dict(vectors=len(rows),valid=accepted,rejected=len(rows)-accepted)
    (BUILD/'result.json').write_text(json.dumps(report,indent=2));print('PASS',report)
if __name__=='__main__':main()
