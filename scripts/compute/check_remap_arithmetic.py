"""Independent binary32 remap arithmetic regression, including gradual underflow."""
from pathlib import Path
import os, subprocess, sys
ROOT=Path(__file__).resolve().parents[2]
BUILD=ROOT/'build/compute/remap_arithmetic'
sys.path.insert(0,str(ROOT/'scripts/common'))
from tools import modelsim_bin
BIN=modelsim_bin()
def run(args):
 p=subprocess.run([str(x) for x in args],cwd=BUILD,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,errors='replace',timeout=180)
 if p.returncode:raise RuntimeError(p.stdout)
 return p.stdout
BUILD.mkdir(parents=True,exist_ok=True)
run([sys.executable,ROOT/'data/generators/compute/generate_fp32_arithmetic.py','--out',BUILD])
if not (BUILD/'work').exists():run([BIN/'vlib.exe','work'])
run([BIN/'vlog.exe','-sv','+incdir+'+str(ROOT/'rtl/include'),ROOT/'rtl/compute/float/fp_divsqrt.v',ROOT/'rtl/compute/float/fp_operator.v',ROOT/'rtl/compute/float/fp32_service.v',ROOT/'tb/compute/remap/tb_fp32.sv'])
(BUILD/'run.do').write_text('onerror {quit -code 1 -f}\nonbreak {quit -code 1 -f}\nrun 1 ms\nif {[examine -radix unsigned /tb_fp32/finished]!=1 || [examine -radix unsigned /tb_fp32/i]!=2500} {quit -code 1 -f}\necho "PASS FP32: 2500 independent cases"\nquit -code 0 -f\n')
s=run([BIN/'vsim.exe','-c','-voptargs=+acc','work.tb_fp32','-do','run.do'])
(BUILD/'simulation.log').write_text(s,encoding='utf-8')
if 'PASS FP32:' not in s or '** Fatal:' in s or '** Error:' in s:raise RuntimeError(s)
print('PASS remap arithmetic: 2500 independent binary32 cases')
