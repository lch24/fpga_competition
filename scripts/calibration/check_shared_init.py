"""Initializer numerical/protocol regression using its real seven-lane FP pool."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'common'))
from tools import modelsim_bin

import subprocess
ROOT=Path(__file__).resolve().parents[2];BUILD=ROOT/'build/system/shared_init';MS=modelsim_bin()
def main():
 BUILD.mkdir(parents=True,exist_ok=True)
 tb=(ROOT/'tb/calibration/tb_init_controller.sv').read_text(encoding='utf-8')
 declarations='wire [6:0] fv,fr,fa,sv,sr;wire [34:0] fo;wire [447:0] fpa,fpb;wire [63:0] value;wire [4:0] flags;\n'
 declarations+='fp_calibration_pool #(.CLIENTS(7)) pool(.clk(clk),.rst_n(rst_n),.c_req_valid(fv),.c_req_ready(fr),.c_req_op(fo),.c_req_a(fpa),.c_req_b(fpb),.c_active(fa),.c_rsp_valid(sv),.c_rsp_ready(sr),.result(value),.flags(flags));\n'
 conn='.shared_req_valid(fv),.shared_req_ready(fr),.shared_req_op(fo),.shared_req_a(fpa),.shared_req_b(fpb),.shared_active(fa),.shared_rsp_valid(sv),.shared_rsp_ready(sr),.shared_rsp_result(value),.shared_rsp_flags(flags),'
 assert 'init_controller dut(' in tb
 tb=tb.replace('init_controller dut(',declarations+'init_controller #(.FP_SHARED(1)) dut('+conn)
 tb=tb.replace('../../../data/calibration/init_controller_vectors.txt',(ROOT/'data/calibration/init_controller_vectors.txt').as_posix())
 (BUILD/'tb.sv').write_text(tb,encoding='utf-8')
 def run(tool,*args):
  p=subprocess.run([str(MS/(tool+'.exe')),*args],cwd=BUILD,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
  (BUILD/(tool+'.log')).write_text(p.stdout,encoding='utf-8')
  if p.returncode:raise RuntimeError(p.stdout[-4000:])
  return p.stdout
 if not (BUILD/'work').exists():run('vlib','work')
 manifest=[]
 for line in (ROOT/'parameter/files.f').read_text().splitlines():
  if not line.strip():continue
  manifest.append('+incdir+'+(ROOT/'parameter'/line[8:]).as_posix() if line.startswith('+incdir+') else (ROOT/'parameter'/line).as_posix())
 (BUILD/'rtl.f').write_text('\n'.join(line if line.startswith(chr(34)) else chr(34)+line+chr(34) for line in manifest)+'\n')
 run('vlog','-sv','-f','rtl.f','tb.sv')
 output=run('vsim','-c','-voptargs=+acc','work.tb_init_controller','-do','do {'+(ROOT/'scripts/calibration/run_init_controller.do').as_posix()+'}')
 assert 'INIT_CONTROLLER_PASS cases=9 protocols=3 errors=0' in output,output[-4000:]
 print('PASS shared init: 9 numerical cases, 3 protocol cases')
if __name__=='__main__':main()
