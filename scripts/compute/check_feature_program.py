"""Bit-exact feature program versus independent ordered binary64 arithmetic."""
from pathlib import Path
import json,random,struct,subprocess,sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts/common'))
from tools import modelsim_bin
out=ROOT/'build/compute/feature_program';out.mkdir(parents=True,exist_ok=True)
# Reuse deterministic inputs, not outputs or a possibly stale build artifact.
from check_tensor_shared import tensor_cases
def b64(x):return struct.pack('>d',x).hex()
def round32(x):return struct.unpack('>f',struct.pack('>f',x))[0]
def b32(x):return struct.pack('>f',x).hex()
rng=random.Random(10613);rows=[]
for a,b,c,bx,by in tensor_cases():
    x,y=(round32(rng.uniform(-2000,2000)) for _ in range(2))
    det=a*c-b*b;trace=a+c;ok=not(trace<1e-8 or det<=(1e-5*trace)*trace)
    dx=(c*bx-b*by)/det if ok else 0.;dy=(a*by-b*bx)/det if ok else 0.
    nx=round32(x+dx);ny=round32(y+dy);conv=dx*dx+dy*dy
    rows.append(str(int(ok))+''.join(map(b64,(a,b,c,bx,by)))+b32(x)+b32(y)+b64(dx)+b64(dy)+b32(nx)+b32(ny)+b64(conv))
(out/'vectors.hex').write_text('\n'.join(rows)+'\n')
ms=modelsim_bin()
def tool(name,*args):
    with (out/(name+'.log')).open('w') as log:
        result=subprocess.run([str(ms/(name+'.exe')),*map(str,args)],cwd=out,stdout=log,stderr=subprocess.STDOUT)
    if result.returncode:raise RuntimeError((out/(name+'.log')).read_text(errors='replace')[-4000:])
if not(out/'work').exists():tool('vlib','work')
sources=['rtl/compute/float/fp_divsqrt.v','rtl/compute/float/calib_alu.v','rtl/compute/float/fp_math_program.v',
         'rtl/compute/float/fp_operator.v','rtl/compute/service/fp_calibration_pool.v',
         'rtl/compute/service/feature_program.v','tb/compute/tb_feature_program.sv']
tool('vlog','-sv','+incdir+'+str(ROOT/'rtl/include'),'+incdir+'+str(ROOT/'rtl/compute/float'),*(ROOT/p for p in sources))
(out/'run.do').write_text('''onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set n [examine -radix unsigned /tb_feature_program/checked]
 set c [examine -radix unsigned /tb_feature_program/other_checked]
 set r [examine -radix unsigned /tb_feature_program/resets]
 if {[examine /tb_feature_program/finished]!=1 || $n!=COUNT || $c<1 || $r!=5 || [examine /tb_feature_program/errors]!=0} {quit -code 1 -f}
 echo "FEATURE_PASS cases=$n concurrent=$c resets=$r max_cycles=[examine -radix unsigned /tb_feature_program/max_cycles]"
 quit -code 0 -f
}
run -all
'''.replace('COUNT',str(len(rows))))
tool('vsim','-c','-voptargs=+acc',f'-gCOUNT={len(rows)}','work.tb_feature_program','-do','run.do')
log=(out/'vsim.log').read_text(errors='replace')
assert '# FEATURE_PASS cases=' in log
print(next(line for line in log.splitlines() if line.startswith('# FEATURE_PASS cases=')))
