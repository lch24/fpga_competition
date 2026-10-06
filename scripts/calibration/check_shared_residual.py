"""Exercise calib_top's actual residual mux/pool with existing numerical vectors.
Controllers are forced at the service boundary, arithmetic and corner read mux
are real. Alternates LM/check ownership between jobs; includes reset and stalls.
This is a scoped integration test, not a full LM end-to-end regression.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'common'))
from tools import modelsim_bin

import re,subprocess
ROOT=Path(__file__).resolve().parents[2];BUILD=ROOT/'build/system/shared_residual'
MS=modelsim_bin()
def main():
 BUILD.mkdir(parents=True,exist_ok=True)
 s=(ROOT/'rtl/compute/geometry/residual_engine.v').read_text(encoding='utf-8')
 header=s[s.index('module residual_engine'):s.index('\n);')+3]
 ports=re.findall(r'\b(input|output)\s+(?:wire|reg)\s*(\[[^\]]+\])?\s*(\w+)',header)
 service=[p for p in ports if not p[2].startswith('shared_') and p[2]!='clk']
 wrapper=header.replace('module residual_engine','module probe_shared_residual',1).replace('output reg','output wire')
 wrapper+='\nreg owner=0; always @(posedge clk) if(!rst_n)owner<=0;else if(rsp_valid&&rsp_ready)owner<=!owner;\n'
 wrapper+='calib_top dut(.clk(clk),.rst_n(rst_n),.collect_valid(1\'b0),.cmd_valid(1\'b0));\n'
 wrapper+='initial begin\nforce dut.compute_rst_n=rst_n;\nforce dut.pc=owner?4\'d11:4\'d8;\n'
 for d,w,n in service:
  if d=='input':
   for phase in ('lm','check'):wrapper+=f'force dut.{phase}_ext_{n}={n};\n'
 wrapper+='end\n'
 for d,w,n in service:
  if d=='output':wrapper+=f'assign {n}=owner?dut.check_ext_{n}:dut.lm_ext_{n};\n'
 wrapper+='always @(posedge clk)if(rst_n)begin\n'
 for name in ('cmd_ready','data_valid','rsp_valid','point_rd_en'):
  wrapper+=f'if(owner?dut.lm_ext_{name}:dut.check_ext_{name})$fatal(1,"inactive owner sees {name}");\n'
 wrapper+='end\nendmodule\n'
 (BUILD/'probe.sv').write_text('`include "calib_defs.vh"\n'+wrapper,encoding='utf-8')
 tb=(ROOT/'tb/compute/tb_residual_engine.sv').read_text(encoding='utf-8').replace('residual_engine dut(','probe_shared_residual dut(')
 tb=tb.replace('../../../data/calibration/residual_engine_vectors.txt',(ROOT/'data/calibration/residual_engine_vectors.txt').as_posix())
 # Full-feature shared FP service has a different latency from standalone
 # feature-pruned cores. Still require reaching the actual cancellation point.
 tb=tb.replace('cycles<10000','cycles<200000')
 (BUILD/'tb.sv').write_text(tb,encoding='utf-8')
 def run(tool,*args):
  p=subprocess.run([str(MS/(tool+'.exe')),*args],cwd=BUILD,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
  (BUILD/(tool+'.log')).write_text(p.stdout,encoding='utf-8')
  if p.returncode:raise RuntimeError(p.stdout[-5000:])
  return p.stdout
 if not (BUILD/'work').exists():run('vlib','work')
 manifest=[]
 for line in (ROOT/'parameter/files.f').read_text().splitlines():
  if not line.strip():continue
  if line.startswith('+incdir+'):manifest.append('+incdir+'+(ROOT/'parameter'/line[8:]).as_posix())
  else:manifest.append((ROOT/'parameter'/line).as_posix())
 (BUILD/'rtl.f').write_text('\n'.join(line if line.startswith(chr(34)) else chr(34)+line+chr(34) for line in manifest)+'\n')
 run('vlog','-sv','-f','rtl.f','probe.sv','tb.sv')
 output=run('vsim','-c','-voptargs=+acc','work.tb_residual_engine','-do','do {'+(ROOT/'scripts/compute/run_residual_engine.do').as_posix()+'}')
 assert 'RESIDUAL_ENGINE_PASS cases=49 protocols=4 errors=0' in output,output[-5000:]
 print('PASS shared residual: 49 numeric cases, 4 protocol cases, alternating owners')
if __name__=='__main__':main()
