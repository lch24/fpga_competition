"""FP64 tensor solver sharing: independent ordered arithmetic and CE/reset checks."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'common'))
from tools import modelsim_bin

import random,struct,subprocess,json,math
ROOT=Path(__file__).resolve().parents[2]
BUILD=ROOT/'build/system/tensor_shared'
MS=modelsim_bin()
def bits(x):return struct.unpack('>Q',struct.pack('>d',x))[0]
def tensor_cases():
    rng=random.Random(61007)
    cases=[(0.,0.,0.,0.,0.),(1.,1.,1.,1.,1.),(1e-10,0.,1e-10,0.,0.),(1.,0.,1.,2.,-3.)]
    for i in range(600):
        scale=2.**rng.randrange(-12,20)
        a=rng.uniform(.1,100)*scale;c=rng.uniform(.1,100)*scale
        b=rng.uniform(-.95,.95)*(a*c)**.5
        if i%11==0:b=(a*c)**.5
        if i%13==0:b=0.
        cases.append((a,b,c,rng.uniform(-10,10)*scale,rng.uniform(-10,10)*scale))
    # Probe both sides of the conditioning and minimum-trace cutoffs.
    for scale in (2.**-20,1.,2.**20):
        boundary=math.sqrt(1.-4e-5)*scale
        for b in (math.nextafter(boundary,-math.inf),boundary,math.nextafter(boundary,math.inf)):
            cases.append((scale,b,scale,scale,-scale))
    for a in (math.nextafter(5e-9,0.),5e-9,math.nextafter(5e-9,math.inf)):
        cases.append((a,0.,a,a,-a))
    return cases

def main():
    BUILD.mkdir(parents=True,exist_ok=True)
    rows=[];accepted=0
    for a,b,c,bx,by in tensor_cases():
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
    sources += [ROOT/'rtl/compute/service/feature_program.v',ROOT/'rtl/compute/float/calib_alu.v',ROOT/'rtl/compute/float/fp_divsqrt.v',ROOT/'rtl/image/kernels/tensor_solve.v',ROOT/'tb/compute/tb_tensor_shared.sv']
    run('vlog','-sv','+incdir+'+str(ROOT/'rtl/compute/float'),'+incdir+'+str(ROOT/'rtl/include'),*map(str,sources))
    (BUILD/'run.do').write_text('onfinish stop\nonerror {quit -code 1 -f}\nonbreak {\n'
        f'if {{[examine /tb_tensor_shared/finished]!=1 || [examine -radix unsigned /tb_tensor_shared/checked]!={len(rows)}}} {{quit -code 1 -f}}\n'
        'echo "PASS tensor shared cycles=[examine -radix unsigned /tb_tensor_shared/total_cycles]"\nquit -code 0 -f\n}\nrun -all\n')
    output=run('vsim','-c','-voptargs=+acc=rn',f'-gCOUNT={len(rows)}','work.tb_tensor_shared','-do','run.do')
    assert 'PASS tensor shared cycles=' in output and '** Fatal:' not in output,output[-5000:]
    report=dict(vectors=len(rows),valid=accepted,rejected=len(rows)-accepted)
    (BUILD/'result.json').write_text(json.dumps(report,indent=2));print('PASS',report)
if __name__=='__main__':main()
