"""Independent exact-integer IEEE addition reference, both widths and stalls."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'common'))
from tools import modelsim_bin

import random,subprocess,json,struct
from fractions import Fraction
ROOT=Path(__file__).resolve().parents[2]
MS=modelsim_bin()
def reference(a,b,bits):
    f,e=(52,11) if bits==64 else (23,8)
    mask=(1<<f)-1;em=(1<<e)-1;sign=1<<(bits-1)
    def unpack(x):
        ex=(x>>f)&em;m=x&mask
        return ex, m|(1<<f if ex else 0)
    ea,ma=unpack(a);eb,mb=unpack(b)
    if ea==em or eb==em:
        if (ea==em and a&mask) or (eb==em and b&mask) or (ea==eb==em and (a^b)&sign):return (em<<f)|(1<<(f-1))
        return (a if ea==em else b)&(sign|(em<<f))
    # Exact signed integer, in units of the smallest subnormal.
    x=(-1 if a&sign else 1)*ma*(1<<max(0,ea-1))
    y=(-1 if b&sign else 1)*mb*(1<<max(0,eb-1))
    z=x+y
    if not z:return sign if a&b&sign else 0
    s=sign if z<0 else 0;z=abs(z)
    shift=max(0,z.bit_length()-f-1)
    q,r=divmod(z,1<<shift)
    if shift and (r>(1<<(shift-1)) or (r==(1<<(shift-1)) and q&1)):q+=1
    if q>=(1<<(f+1)):q>>=1;shift+=1
    ex=shift+1 if q>=(1<<f) else 0
    return s|(em<<f) if ex>=em else s|(ex<<f)|(q&mask)
def main():
  report={}
  for bits,op in ((32,0),(64,0),(32,1),(32,2),(64,3)):
    build=ROOT/f'build/system/add_pipeline_{bits}_op{op}';build.mkdir(parents=True,exist_ok=True)
    f,e=(52,11) if bits==64 else (23,8);rng=random.Random(61008+bits)
    sign=1<<(bits-1);em=(1<<e)-1
    edges=[0,1,2,(1<<f)-1,1<<f,(1<<f)+1,((em//2)<<f),((em//2)<<f)+1,((em-1)<<f)|((1<<f)-1),em<<f,(em<<f)+1]
    edges+= [v|sign for v in edges]
    pairs=[(a,b) for a in edges for b in edges]
    pairs += [(rng.getrandbits(bits),rng.getrandbits(bits)) for _ in range(10000)]
    # Cancellation, half-ULP boundaries, exponent differences spanning mantissa.
    for _ in range(2000):
      a=rng.randrange(1,em)<<f|rng.getrandbits(f);b=a+rng.randrange(-8,9)
      pairs.append((a,b^sign))
    if op==2:pairs=[(float_bits(2.**rng.uniform(-5,5)),0) for _ in range(1000)]+[(0x3f800000+i,0) for i in range(-20,21)]
    ref=(lambda a,b:reference(a,b,bits)) if op==0 else (division if op==1 else log_poly if op==2 else lambda a,b:reference(a,b^(1<<63),64))
    rows=[f'{a:0{bits//4}x}{b:0{bits//4}x}{ref(a,b):0{bits//4}x}' for a,b in pairs]
    (build/'add_vectors.hex').write_text('\n'.join(rows)+'\n')
    def run(tool,*args):
      p=subprocess.run([str(MS/(tool+'.exe')),*args],cwd=build,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
      (build/(tool+'.log')).write_text(p.stdout)
      if p.returncode:raise RuntimeError(p.stdout[-5000:])
      return p.stdout
    if not (build/'work').exists():run('vlib','work')
    sources=list((ROOT/'rtl/compute/float/stream').glob('*.v'))+list((ROOT/'rtl/common').glob('*.v'))
    sources += [ROOT/'rtl/compute/float/fp_divsqrt.v',ROOT/'tb/compute/tb_add_pipeline.sv']
    run('vlog','-sv','+incdir+'+str(ROOT/'rtl/compute/float'),'+incdir+'+str(ROOT/'rtl/include'),*map(str,sources))
    (build/'run.do').write_text('onfinish stop\nonerror {quit -code 1 -f}\nonbreak {\n'+f'if {{[examine /tb_add_pipeline/finished]!=1 || [examine -radix unsigned /tb_add_pipeline/checked]!={len(rows)}}} {{quit -code 1 -f}}\n'+'echo "PASS add pipeline checked=[examine -radix unsigned /tb_add_pipeline/checked]"\nquit -code 0 -f\n}\nrun -all\n')
    output=run('vsim','-c','-voptargs=+acc=rn',f'-gBITS={bits}',f'-gCOUNT={len(rows)}',f'-gOP={op}','work.tb_add_pipeline','-do','run.do')
    assert 'PASS add pipeline checked=' in output and '** Fatal' not in output,output[-5000:]
    report[f'{bits}_op{op}']=len(rows);print('PASS',bits,'op',op,len(rows),flush=True)
  (ROOT/'build/system/add_pipeline_result.json').write_text(json.dumps(report,indent=2))
def float_bits(x):return struct.unpack('>I',struct.pack('>f',x))[0]
def float_value(x):return struct.unpack('>f',struct.pack('>I',x))[0]
def log_poly(a,b):
    rnd=lambda x:float_value(float_bits(x))
    x=float_value(a);z=rnd(rnd(x-1)/rnd(x+1));z2=rnd(z*z)
    p=float_value(0x3d9d89d9)
    for c in (0x3d888889,0x3dba2e8c,0x3de38e39,0x3e124925,0x3e4ccccd,0x3eaaaaab,0x3f800000):p=rnd(float_value(c)+rnd(z2*p))
    return float_bits(rnd(2*rnd(z*p)))
def division(a,b):
    ea=(a>>23)&255;eb=(b>>23)&255;fa=a&0x7fffff;fb=b&0x7fffff;s=(a^b)&0x80000000
    if (ea==255 and fa) or (eb==255 and fb) or (ea==eb==255) or (a&0x7fffffff==0 and b&0x7fffffff==0):return 0x7fc00000
    if ea==255 or b&0x7fffffff==0:return s|0x7f800000
    if eb==255 or a&0x7fffffff==0:return s
    value=Fraction(fa|(0x800000 if ea else 0),fb|(0x800000 if eb else 0))*Fraction(2)**(max(1,ea)-max(1,eb))
    ex=value.numerator.bit_length()-value.denominator.bit_length()
    if value<Fraction(2)**ex:ex-=1
    scaled=value*Fraction(2)**(23-max(ex,-126));q,r=divmod(scaled.numerator,scaled.denominator)
    if 2*r>scaled.denominator or (2*r==scaled.denominator and q&1):q+=1
    if q>=1<<24:q>>=1;ex+=1
    if ex>127:return s|0x7f800000
    field=max(1,ex+127) if q>=1<<23 else 0
    return s|(field<<23)|(q&0x7fffff)
if __name__=='__main__':main()
