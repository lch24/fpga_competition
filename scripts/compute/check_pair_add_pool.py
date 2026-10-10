"""FP32 paired shared adder: ordered IEEE rounding, contention and no HOL wait."""
from pathlib import Path
import sys,random,struct,subprocess
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts/common'))
from tools import modelsim_bin
BUILD=ROOT/'build/compute/pair_add_pool'
def main():
 BUILD.mkdir(parents=True,exist_ok=True);rng=random.Random(62108)
 def f32(x):return struct.unpack('>f',struct.pack('>f',x))[0]
 def bits(x):return struct.unpack('>I',struct.pack('>f',x))[0]
 rows=[]
 for i in range(512):
  sub=i%2;pair=(i//2)%2
  a=[f32(rng.uniform(-1,1)*2**rng.randrange(-60,60)) for _ in range(2)]
  b=[f32(rng.uniform(-1,1)*2**rng.randrange(-60,60)) for _ in range(2)]
  if i%9==0:b[0]=-a[0] if not sub else a[0]
  # Exact sum of binary32 operands fits Python binary64 for relevant rounding.
  r=[bits(a[k]-b[k] if sub else a[k]+b[k]) for k in range(2)]
  aa=(bits(a[1])<<32)|bits(a[0]);bb=(bits(b[1])<<32)|bits(b[0]);rr=(r[1]<<32 if pair else 0)|r[0]
  rows.append((sub<<193)|(pair<<192)|(aa<<128)|(bb<<64)|rr)
 (BUILD/'vectors.hex').write_text(''.join(f'{x:049x}\n' for x in rows))
 ms=modelsim_bin()
 def run(tool,*args):
  p=subprocess.run([str(ms/(tool+'.exe')),*args],cwd=BUILD,capture_output=True,text=True)
  (BUILD/(tool+'.log')).write_text(p.stdout+p.stderr)
  if p.returncode:raise RuntimeError((p.stdout+p.stderr)[-3000:])
 if not (BUILD/'work').exists():run('vlib','work')
 sources=list((ROOT/'rtl/compute/float/stream').glob('*.v'))+list((ROOT/'rtl/common').glob('*.v'))
 sources +=[ROOT/'rtl/compute/service/fp32_pair_add_pool.v',ROOT/'tb/compute/tb_fp32_pair_add_pool.sv']
 run('vlog','-sv','+incdir+'+str(ROOT/'rtl/include'),*map(str,sources))
 (BUILD/'run.do').write_text('onfinish stop\nonerror {quit -code 1 -f}\nonbreak {resume}\nrun -all\n'
  'if {[examine /tb_fp32_pair_add_pool/finished] != 1 || [examine -radix unsigned /tb_fp32_pair_add_pool/checked] != 512} {quit -code 1 -f}\n'
  'echo PAIR_ADD_POOL_PASS\nquit -code 0 -f\n')
 run('vsim','-c','-voptargs=+acc','tb_fp32_pair_add_pool','-do','run.do')
 print('PAIR_ADD_POOL_PASS jobs=512 reset=1 clients=4 CE/backpressure/no-HOL')
if __name__=='__main__':main()
