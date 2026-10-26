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
           'lm':('lm_controller',9,None),'lm_service':('lm_service',3,None),
           'lm_fisheye':('lm_fisheye',1,None),'lm_replay':('lm_controller',1,None),
           'lm_lens':('lm_lens',2,None)}
    name,cases,protocols=names[phase]
    quick='--quick' in sys.argv
    policy='--policy' in sys.argv
    if policy and (phase!='validate' or quick):raise ValueError('--policy is for validate only')
    policy_rows=[0,1,9,17,26,38,43,48,55,60,61,62,63,64]
    if policy:cases=len(policy_rows)
    if quick and phase not in ('init','validate'):
        raise ValueError('--quick is supported by init and validate only')
    ms=modelsim_bin();out=ROOT/'build/calibration_engine'/(phase+('_quick' if quick else '_policy' if policy else ''))
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
    elif phase in ('lm_replay','lm_lens'):
        import json
        from generate_lm_fisheye import bits, bits32
        report=json.loads((ROOT/'build/debug/calibration_replay.json').read_text())
        source=Path(report['source'])
        raw=json.loads((source if source.is_absolute() else ROOT/source).read_text())
        defines=[f'+define+PAR_VIEWS={report["views"]}',f'+define+PAR_BOARD_ROWS={report["rows"]}',f'+define+PAR_BOARD_COLS={report["cols"]}']
        def pack(values):return ''.join(f'{bits(v):016x}' for v in reversed(values))
        points=''.join(f'{bits32(e["xy"][1]):08x}{bits32(e["xy"][0]):08x}' for e in reversed(raw['corners']))
        ref=report['regularized_full']
        fixture=f'2 {int(ref["converged"])} {ref["accepted"]} {bits(ref["rms"]**2*len(raw["corners"])):016x} {pack(report["initial"])} {points} {pack(ref["state"])}\n'
        if phase=='lm_lens':
            synthetic=(ROOT/'data/calibration/lm_fisheye_vectors.txt').read_text().split()
            fixture='640 480 '+' '.join(synthetic[3:])+'\n'+f'{report["width"]} {report["height"]} '+' '.join(fixture.split()[3:])+'\n'
        (out/f'{name}_vectors.txt').write_text(fixture)
        tb=ROOT/f'tb/calibration/tb_{name}.sv'
    else:
        tb=ROOT/f'tb/calibration/tb_{name}.sv'
        shutil.copyfile(ROOT/f'data/calibration/{name}_vectors.txt',out/f'{name}_vectors.txt')
        if policy:
            lines=(out/f'{name}_vectors.txt').read_text().splitlines()
            (out/f'{name}_vectors.txt').write_text('\n'.join(lines[i] for i in policy_rows)+'\n')
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
 if {{$tick % 100 == 0}} {{echo "PROGRESS phase={phase} simulated_ms=$tick"}}
 if {{[examine -radix unsigned {path}/done] == 1}} {{break}}
}}
if {{{checks}}} {{quit -code 1 -f}}
echo {marker}
quit -code 0 -f
'''
    (out/'run.do').write_text(script)
    extra=[f'-gREPLAY=1',f'-gWIDTH={report["width"]}',f'-gHEIGHT={report["height"]}'] if phase=='lm_replay' else []
    visibility=f'-voptargs=+acc=r+/tb_{name} -O5' if phase in ('lm_replay','lm_lens','lm_fisheye') else '-voptargs=+acc'
    tool('vsim','-c',visibility,*extra,*(['-gQUICK=1'] if quick else []),f'work.tb_{name}','-do','do run.do')
    log=(out/'vsim.log').read_text(encoding='utf-8',errors='replace')
    assert marker in log
    print(marker+(' quick' if quick else '')+f' cases={1 if quick else cases}')
