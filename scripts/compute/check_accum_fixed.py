"""Exact fixed-point accumulator oracle; randomized windows, stalls and reset."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'common'))
from tools import modelsim_bin

import struct,random,subprocess,json
ROOT=Path(__file__).resolve().parents[2];BUILD=ROOT/'build/system/accum_fixed';MS=modelsim_bin()
bits=lambda x:struct.unpack('>Q',struct.pack('>d',x))[0]
def main():
 BUILD.mkdir(parents=True,exist_ok=True);rng=random.Random(61009);rows=[];total=0;maxrel=0.
 for case in range(100):
  count=[0,1,9,49,225,961][case%6];samples=[];acc=[0]*5;ref=[0.]*5
  for j in range(count):
   x=rng.randint(-15,15);y=rng.randint(-15,15);w=rng.random();gx=rng.uniform(-255,255);gy=rng.uniform(-255,255)
   if case<6:w=1.;gx=255.;gy=-255.
   if case%11==0:w=0.
   samples.append((float(x),float(y),w,gx,gy));qx=int(gx*65536);qy=int(gy*65536);qw=int(w*2**24)
   aa=(qx*qx*qw)>>32;bb=(qx*qy*qw)>>32;cc=(qy*qy*qw)>>32
   vals=(aa,bb,cc,aa*x+bb*y,bb*x+cc*y)
   for k,v in enumerate(vals):acc[k]+=v
   a=w*gx*gx;b=w*gx*gy;c=w*gy*gy
   for k,v in enumerate((a,b,c,a*x+b*y,b*x+c*y)):ref[k]+=v
  fixed=[v/2**24 for v in acc]
  maxrel=max(maxrel,*[abs(a-b)/(1+abs(b)) for a,b in zip(fixed,ref)])
  rows.append(str(count)+' '+''.join(f'{bits(v):016x}' for v in fixed))
  rows.extend(' '.join(f'{bits(v):016x}' for v in sample) for sample in samples);total+=count
 (BUILD/'accum.txt').write_text('\n'.join(rows)+'\n')
 def run(tool,*args):
  p=subprocess.run([str(MS/(tool+'.exe')),*args],cwd=BUILD,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
  (BUILD/(tool+'.log')).write_text(p.stdout)
  if p.returncode:raise RuntimeError(p.stdout[-4000:])
  return p.stdout
 if not (BUILD/'work').exists():run('vlib','work')
 run('vlog','-sv',str(ROOT/'rtl/image/features/subpixel_accum.v'),str(ROOT/'tb/compute/tb_accum_fixed.sv'))
 (BUILD/'run.do').write_text('onfinish stop\nonerror {quit -code 1 -f}\nonbreak {\nif {[examine /tb_accum_fixed/finished]!=1 || [examine -radix unsigned /tb_accum_fixed/checked]!=100} {quit -code 1 -f}\necho "PASS fixed accumulation"\nquit -code 0 -f\n}\nrun -all\n')
 output=run('vsim','-c','-voptargs=+acc=rn','work.tb_accum_fixed','-do','run.do')
 assert 'PASS fixed accumulation' in output and '** Fatal' not in output
 report=dict(windows=100,samples=total,max_normalized_difference=maxrel)
 (BUILD/'result.json').write_text(json.dumps(report,indent=2));print('PASS',report)
if __name__=='__main__':main()
