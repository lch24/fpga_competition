"""Directed checks for the optional artificial-distortion extension. No synthesis."""
from pathlib import Path
import argparse, os, subprocess, re, sys
ROOT=Path(__file__).resolve().parents[2]
def main():
 p=argparse.ArgumentParser();p.add_argument('--modelsim',default=os.environ.get('MODELSIM_BIN'));p.add_argument('--only');p.add_argument('--board',action='store_true');a=p.parse_args()
 if not a.modelsim:p.error('supply --modelsim or MODELSIM_BIN')
 subprocess.run([sys.executable,str(ROOT/'data/generators/demo/generate_coordinates.py')],check=True)
 build=ROOT/'build/demo';build.mkdir(parents=True,exist_ok=True)
 math=['rtl/image/remap/map_coord_core.v','rtl/compute/float/fp32_service.v','rtl/compute/float/fp_divsqrt.v']
 tests={
 'tb_demo_uart':['rtl/debug/board_debug.v','rtl/debug/algorithm_debug.v','rtl/debug/debug_snapshot.v','rtl/common/reset_sync.v','rtl/io/uart/debug_uart.v','tb/debug/tb_board_debug.sv'],
 'tb_demo_buttons':['rtl/control/system/board_buttons.v'],
 'tb_photo_preview':['rtl/control/system/video_board_flow.v','rtl/control/demo/distort_demo_flow.v'],
 'tb_demo_flow':['rtl/control/system/video_board_flow.v','rtl/control/demo/distort_demo_flow.v'],
 'tb_demo_map':math,
 'tb_demo_sequence':['rtl/control/system/vision_sequence.v','tb/system/tb_vision_manual.sv'],
 'tb_packed_map':math+['rtl/compute/geometry/packed_map_builder.v','rtl/image/remap/sample_coord.v','rtl/image/remap/output_writer.v'],
 'tb_demo_packed_map':math+['rtl/compute/geometry/packed_map_builder.v','rtl/image/remap/sample_coord.v','rtl/image/remap/output_writer.v','tb/system/tb_packed_map.sv'],
 'tb_video_flow':['rtl/control/system/video_board_flow.v'],
 'tb_realtime_remap':['rtl/image/remap/'+n+'.v' for n in ('realtime_remap','video_map_prefetch','video_grid_prefetch','video_neighbor_cache')]+['rtl/image/kernels/video_bilinear_pipe.v','rtl/video/dma/video_frame_writer.v','rtl/memory/ddr/video_hmic_arbiter.v'],
 'tb_vision_build_only':['rtl/control/system/vision_sequence.v','tb/system/tb_vision_manual.sv'],
 }
 if a.board:
  r=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'scripts/system/run_checks.ps1'),'-ModelSimBin',a.modelsim,'-Board','-OnlyTest','tb_board_buttons'],cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,errors='replace')
  (build/'board_compile.log').write_text(r.stdout,encoding='utf-8');assert r.returncode==0,r.stdout[-2000:]
  for enabled in (0,1):
   r=subprocess.run([str(Path(a.modelsim)/'vsim.exe'),'-c','-voptargs=+acc','-gENABLE_DISTORT_DEMO='+str(enabled),'-do','do {'+(ROOT/'scripts/demo/check_board.tcl').as_posix()+'}','calibrated_view_top'],cwd=ROOT/'build/system',stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,errors='replace',timeout=90)
   (build/('board_'+str(enabled)+'.log')).write_text(r.stdout,encoding='utf-8')
   assert r.returncode==0 and 'PASS board elaboration' in r.stdout,r.stdout[-2000:]
   print('PASS board ENABLE_DISTORT_DEMO='+str(enabled),flush=True)
 if a.only and a.only not in tests:p.error('unknown test '+a.only)
 for name,src in tests.items():
  if a.only and name!=a.only:continue
  directory=build/name;directory.mkdir(exist_ok=True)
  bench=list((ROOT/'tb').rglob(name+'.sv'));assert len(bench)==1
  flag='/'+name+('/test' if name in ('tb_demo_sequence','tb_demo_packed_map','tb_demo_uart') else '/scenario' if name=='tb_vision_build_only' else '')+'/finished'
  driver=directory/'run.do'
  driver.write_text('transcript on\nonfinish stop\nonerror {quit -code 1 -f}\nonbreak {\n if {[examine -radix unsigned '+flag+'] != 1} {quit -code 1 -f}\n echo "PASS '+name+'"\n quit -code 0 -f\n}\nrun -all\n')
  commands=[('vlib',['work']),('vlog',['-sv','+incdir+'+str(ROOT/'rtl/include')]+[str(ROOT/x) for x in src]+[str(bench[0])]),('vsim',['-c','-voptargs=+acc','-do','do {'+driver.as_posix()+'}',name])]
  for tool,args in commands:
   r=subprocess.run([str(Path(a.modelsim)/(tool+'.exe'))]+args,cwd=directory,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,errors='replace',timeout=180)
   (directory/(tool+'.log')).write_text(r.stdout,encoding='utf-8')
   assert r.returncode==0,(name,tool,r.returncode,r.stdout[-3000:])
   if tool=='vsim':assert 'PASS' in r.stdout and not re.search(r'\*\* (?:Fatal|Error)',r.stdout),(name,r.stdout[-3000:])
  print('PASS',name,flush=True)
if __name__=='__main__':main()

