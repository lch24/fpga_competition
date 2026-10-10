"""Compare serial distance RTL to independent binary64 sqrt then FP32 rounding.
Includes the existing C++ vectors, broad IEEE inputs, CE, backpressure/reset.
"""
from pathlib import Path
import math,random,struct,subprocess,sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts/common'))
from tools import modelsim_bin
out=ROOT/'build/compute/hypot_serial';out.mkdir(parents=True,exist_ok=True)
ms=modelsim_bin()
def number(bits):return struct.unpack('<f',struct.pack('<I',bits))[0]
def bits(value):
    try:return struct.unpack('<I',struct.pack('<f',value))[0]
    except OverflowError:return 0x7f800000
def expected(a,b):
    a,b=number(a),number(b)
    if math.isinf(a) or math.isinf(b):return 0x7f800000
    if math.isnan(a) or math.isnan(b):return 0x7fc00000
    return bits(math.sqrt(a*a+b*b))
values=[0,0x80000000,1,0x7fffff,0x800000,0x3f800000,0x40000000,
        0x40400000,0x40800000,0x7f7fffff,0x7f800000,0xff800000,0x7fc00001]
vectors=[(a,b,expected(a,b)) for a in values for b in values]
rng=random.Random(715923)
for i in range(12000):
    a,b=(bits(rng.uniform(-16384,16384)) for _ in range(2)) if i%2 else (rng.getrandbits(32) for _ in range(2))
    vectors.append((a,b,expected(a,b)))
# Original C++ std::hypot(float,float) vectors remain an independent check.
legacy=ROOT/'data/image/test_fp32.bin'
if legacy.exists():
    content=legacy.read_bytes();count=struct.unpack_from('<I',content)[0]
    for i in range(count):
        kind,a,b,e=struct.unpack_from('<4I',content,4+16*i)
        if kind==0:vectors.append((a,b,e))
(out/'vectors.txt').write_text(''.join(f'{a:08x} {b:08x} {e:08x}\n' for a,b,e in vectors))
def tool(name,*args):
    with (out/(name+'.log')).open('w') as log:
        p=subprocess.run([str(ms/(name+'.exe')),*map(str,args)],cwd=out,stdout=log,stderr=subprocess.STDOUT)
    if p.returncode:raise RuntimeError((out/(name+'.log')).read_text(errors='replace')[-4000:])
if not(out/'work').exists():tool('vlib','work')
tool('vlog','-sv',ROOT/'rtl/common/sync_fifo.v',
     ROOT/'rtl/compute/float/stream/fp64_sqrt.v',ROOT/'rtl/compute/float/stream/f64_to_f32.v',
     ROOT/'rtl/compute/float/stream/fp32_hypot.v',ROOT/'rtl/compute/service/hypot_port.v',
     ROOT/'rtl/compute/service/hypot_pool.v',ROOT/'tb/compute/stream/tb_hypot_serial.sv',
     ROOT/'tb/compute/stream/tb_hypot_pool.sv')
def simulate(tb,report):
    script='''onfinish stop
onerror {quit -code 1 -f}
onbreak {
 if {[examine -radix unsigned /TEST/done] != 1 || [examine -radix unsigned /TEST/errors] != 0} {quit -code 1 -f}
 echo HYPOT_PASS
 quit -code 0 -f
}
run -all
quit -code 1 -f
'''
    (out/'run.do').write_text(script.replace('TEST',tb))
    tool('vsim','-c','-voptargs=+acc','work.'+tb,'-do','run.do')
    assert 'HYPOT_PASS' in (out/'vsim.log').read_text(errors='replace')
    print((out/report).read_text())
if '--pool-only' not in sys.argv:simulate('tb_hypot_serial','results.txt')
simulate('tb_hypot_pool','pool_results.txt')
