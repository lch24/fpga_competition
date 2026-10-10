"""Control ISA regression with an independently enumerated service oracle."""
from pathlib import Path
import subprocess,sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts/common'))
from tools import modelsim_bin
BUILD=ROOT/'build/image/detection_program'
def main():
    BUILD.mkdir(parents=True,exist_ok=True);commands=[];cases=[]
    def call(i,arg=0,result=0):commands.append((i<<48)|(arg<<32)|result)
    # Per layer: native rejection, candidate count, ordered grid found, refine valid.
    # Mapping reuses the preceding valid layer; refines may fail without native retry.
    for layers in [[],[(False,3,True,True)],[(False,0,False,False)],
                   [(True,0,False,False)],[(False,2,False,False)],
                   [(False,1,True,True),(False,1,True,True)],
                   [(False,2,True,False),(False,4,True,True)],
                   [(False,2,True,True),(False,2,True,False),(False,1,True,True)]]:
      for dump in (False,True):
        call(9,result=int(not layers))
        if not layers:cases.append((len(commands)<<2)|3);continue
        previous=False
        for level,(reject,n,ordered,valid) in enumerate(layers):
          if previous:
            call(4)
          else:
            call(1);call(16,result=int(reject))
            if not reject:
              call(17);call(18);call(19,result=(n<<16)|(2 if n==0 else 0))
              for i in range(n):call(20,i)
            call(21);call(32,result=int(reject or n==0))
            if not reject and n:
              for i in range(90):call(33,i)
            ordered=ordered and not reject and n>0
            call(34,result=int(not ordered))
            if ordered:call(2)
          if previous or ordered:
            call(3);call(48);call(49);call(50);call(51);call(52)
            previous=valid
          else:previous=False
          call(5,result=int(level==len(layers)-1)|(int(previous)<<1))
        call(6,result=int(not previous))
        if previous:
          call(7,result=int(dump))
          if dump:call(8)
        cases.append((len(commands)<<2)|(1 if previous else 2))
    (BUILD/'commands.hex').write_text(''.join(f'{x:014x}\n' for x in commands))
    (BUILD/'cases.hex').write_text(''.join(f'{x:09x}\n' for x in cases))
    (BUILD/'counts.txt').write_text(f'{len(cases)} {len(commands)}\n')
    ms=modelsim_bin()
    def run(tool,*args):
      p=subprocess.run([str(ms/(tool+'.exe')),*args],cwd=BUILD,capture_output=True,text=True)
      (BUILD/(tool+'.log')).write_text(p.stdout+p.stderr)
      if p.returncode:raise RuntimeError((p.stdout+p.stderr)[-3500:])
    if not (BUILD/'work').exists():run('vlib','work')
    run('vlog','-sv','+incdir+'+str(ROOT/'rtl/include'),str(ROOT/'rtl/control/detection/detection_program.v'),str(ROOT/'rtl/control/detection/detection_flow_control.v'),str(ROOT/'tb/control/tb_detection_program.sv'),str(ROOT/'tb/control/tb_detection_flow_control.sv'))
    (BUILD/'run.do').write_text('onfinish stop\nonerror {quit -code 1 -f}\nonbreak {resume}\nrun -all\n'
       f'if {{[examine /tb_detection_program/finished] != 1 || [examine -radix unsigned /tb_detection_program/checked] != {len(cases)}}} {{quit -code 1 -f}}\n'
       'echo DETECTION_PROGRAM_PASS\nquit -code 0 -f\n')
    run('vsim','-c','-voptargs=+acc','tb_detection_program','-do','run.do')
    print(f'DETECTION_PROGRAM_PASS cases={len(cases)} services={len(commands)} reset=1')
    (BUILD/'program.log').write_text((BUILD/'vsim.log').read_text())
    (BUILD/'flow.do').write_text('onfinish stop\nonerror {quit -code 1 -f}\nonbreak {resume}\nrun -all\n'
       'if {[examine /tb_detection_flow_control/finished] != 1 || [examine -radix unsigned /tb_detection_flow_control/checked] != 4} {quit -code 1 -f}\n'
       'echo DETECTION_FLOW_PASS\nquit -code 0 -f\n')
    run('vsim','-c','-voptargs=+acc','tb_detection_flow_control','-do','flow.do')
    print('DETECTION_FLOW_PASS cases=4 depth=3 CE/backpressure/fallback/map/dump/restart')
if __name__=='__main__':main()
