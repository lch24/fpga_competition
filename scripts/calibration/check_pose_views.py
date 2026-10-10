"""Pose wrapper multi-view regression, using existing independent references.

Each extra view duplicates the last reference homography and its independently
computed pose. This tests addressing/publication for 4..16 views, not geometric
conditioning of a full multi-view calibration with duplicate images.
"""
import argparse
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'common'))
from tools import modelsim_bin

ROOT=Path(__file__).resolve().parents[2]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--views',type=int,default=4)
    args=parser.parse_args()
    if not 4<=args.views<=16:raise ValueError('expected 4..16 views')
    build=ROOT/f'build/system/pose_views_{args.views}';build.mkdir(parents=True,exist_ok=True)
    rows=[]
    for line in (ROOT/'data/calibration/pose_init_vectors.txt').read_text().splitlines():
        status,width,height,h,k,state=line.split();h=int(h,16);state=int(state,16)
        last_h=(h>>(18*64))&((1<<(9*64))-1)
        last_pose=(state>>(21*64))&((1<<(6*64))-1)
        for view in range(3,args.views):
            h|=last_h<<(view*9*64);state|=last_pose<<((9+view*6)*64)
        rows.append(f'{status} {width} {height} {h:x} {k} {state:x}')
    data=build/'vectors.txt';data.write_text('\n'.join(rows)+'\n')
    bench=(ROOT/'tb/calibration/tb_pose_init.sv').read_text(encoding='utf-8')
    bench=bench.replace('../../../data/calibration/pose_init_vectors.txt',data.as_posix())
    (build/'tb.sv').write_text(bench,encoding='utf-8')
    manifest=[]
    for line in (ROOT/'rtl/files.f').read_text().splitlines():
        if line:manifest.append('+incdir+'+(ROOT/line[8:]).as_posix() if line.startswith('+incdir+') else (ROOT/line).as_posix())
    (build/'rtl.f').write_text('\n'.join('"'+x+'"' for x in manifest)+'\n')
    ms=modelsim_bin()
    def run(tool,*arguments):
        result=subprocess.run([str(ms/(tool+'.exe')),*arguments],cwd=build,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        (build/(tool+'.log')).write_text(result.stdout,encoding='utf-8')
        if result.returncode:raise RuntimeError(result.stdout[-5000:])
        return result.stdout
    if not (build/'work').exists():run('vlib','work')
    run('vlog','-sv',f'+define+PAR_VIEWS={args.views}','+define+PAR_BOARD_ROWS=6','+define+PAR_BOARD_COLS=7','-f','rtl.f','tb.sv')
    output=run('vsim','-c','-voptargs=+acc','work.tb_pose_init','-do','do {'+(ROOT/'scripts/calibration/run_pose_init.do').as_posix()+'}')
    if 'POSE_INIT_PASS cases=54 protocols=3 errors=0' not in output:raise RuntimeError(output[-4000:])
    print(f'POSE_VIEWS_PASS views={args.views} board=6x7: '+(build/'pose_init_results.txt').read_text().strip())

if __name__=='__main__':main()
