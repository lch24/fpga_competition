"""Common ModelSim runner for calibration programs; no machine-specific paths."""
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'scripts/common'))
from tools import modelsim_bin

def run(phase):
    names={'init':('init_controller',9,3),'validate':('validate_result',66,5),
           'lm':('lm_controller',9,None),'lm_service':('lm_service',3,None)}
    name,cases,protocols=names[phase]
    quick='--quick' in sys.argv
    if quick and phase not in ('init','validate'):
        raise ValueError('--quick is supported by init and validate only')
    ms=modelsim_bin();out=ROOT/'build/calibration_engine'/(phase+('_quick' if quick else ''))
    out.mkdir(parents=True,exist_ok=True)
    def tool(name,*args):
        with (out/(name+'.log')).open('w',encoding='utf-8') as log:
            result=subprocess.run([str(ms/(name+'.exe')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT)
        if result.returncode:
            raise RuntimeError((out/(name+'.log')).read_text(encoding='utf-8',errors='replace')[-4000:])
    if not (out/'work').exists():tool('vlib','work')
    from build_rom import combined_rom
    entries=combined_rom()
    if phase in ('init','validate'):
        from assemble import assemble
        listing=(ROOT/f'build/calibration_engine/{phase}.asm').read_text(encoding='utf-8')
        _,labels=assemble(listing,4096)
        symbol,label=('POSE_ENTRY','pose_view') if phase=='init' else ('MAP_ENTRY','map_loop')
        (out/f'{phase}_test_defs.vh').write_text(f'localparam {symbol}={entries[phase.upper()]+labels[label]};\n')
    manifest=[]
    for line in (ROOT/'rtl/files.f').read_text().splitlines():
        if not line.strip() or line.startswith('#'):continue
        manifest.append('+incdir+'+(ROOT/line[8:]).as_posix() if line.startswith('+incdir+') else (ROOT/line).as_posix())
    (out/'rtl.f').write_text('\n'.join('"'+line+'"' for line in manifest)+'\n')
    defines=[]
    if phase=='lm_service':
        tb=ROOT/'tb/control/calibration/engine/tb_lm_service.sv'
        defines=['+define+PAR_BOARD_ROWS=2','+define+PAR_BOARD_COLS=2']
    else:
        tb=ROOT/f'tb/calibration/tb_{name}.sv'
        shutil.copyfile(ROOT/f'data/calibration/{name}_vectors.txt',out/f'{name}_vectors.txt')
    tool('vlog','-sv',*defines,'-f','rtl.f',str(tb))
    marker=phase.upper()+'_PROGRAM_PASS'
    if '--compile-only' in sys.argv:print(phase.upper()+'_COMPILE_PASS');return
    path=f'/tb_{name}'
    checks=f'[examine -radix unsigned {path}/done] != 1 || [examine -radix unsigned {path}/errors] != 0 || [examine -radix unsigned {path}/cases] != {1 if quick else cases}'
    if protocols is not None:checks+=f' || [examine -radix unsigned {path}/protocol_cases] != {0 if quick else protocols}'
    script=f'''transcript on
onerror {{quit -code 1 -f}}
onbreak {{quit -code 1 -f}}
for {{set tick 0}} {{$tick < 20000}} {{incr tick}} {{
 run 1 ms
 if {{[examine -radix unsigned {path}/done] == 1}} {{break}}
}}
if {{{checks}}} {{quit -code 1 -f}}
echo {marker}
quit -code 0 -f
'''
    (out/'run.do').write_text(script)
    tool('vsim','-c','-voptargs=+acc',*(['-gQUICK=1'] if quick else []),f'work.tb_{name}','-do','do run.do')
    log=(out/'vsim.log').read_text(encoding='utf-8',errors='replace')
    assert marker in log
    print(marker+(' quick' if quick else '')+f' cases={1 if quick else cases}')
