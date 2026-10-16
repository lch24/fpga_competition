param([string]$IcarusBin=$env:IVERILOG_BIN,[string]$ModelSimBin=$env:MODELSIM_BIN)
# Directed RTL checks for the realtime video path. Runs on Icarus Verilog when
# available, otherwise on ModelSim. Each test elaborates in its own directory so
# that one test's `work` library never leaks into the next.
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$build=Join-Path $root 'build/realtime'
New-Item -ItemType Directory -Force $build | Out-Null

function Resolve-ToolDir([string]$exe){
 $cmd=Get-Command $exe -ErrorAction SilentlyContinue
 if($cmd){return Split-Path $cmd.Source}
 return $null
}

if(!$IcarusBin){$IcarusBin=Resolve-ToolDir 'iverilog.exe'}
if(!$ModelSimBin){$ModelSimBin=Resolve-ToolDir 'vsim.exe'}
if(!$IcarusBin -and !$ModelSimBin){throw "Neither iverilog.exe nor vsim.exe on PATH; pass -IcarusBin or -ModelSimBin"}
$model=$false
if(!$IcarusBin){$model=$true}

$tests=@{
 tb_realtime_remap=@('rtl/image/remap/realtime_remap.v','rtl/image/remap/video_map_prefetch.v','rtl/image/remap/video_neighbor_cache.v','rtl/image/kernels/video_bilinear_pipe.v','rtl/video/dma/video_frame_writer.v','rtl/memory/ddr/video_hmic_arbiter.v')
 tb_realtime_fullframe=@('rtl/image/remap/realtime_remap.v','rtl/image/remap/video_map_prefetch.v','rtl/image/remap/video_neighbor_cache.v','rtl/image/kernels/video_bilinear_pipe.v','rtl/video/dma/video_frame_writer.v','rtl/memory/ddr/video_hmic_arbiter.v')
 tb_video_capture=@('rtl/video/dma/video_capture_hmic.v','rtl/common/async_fifo.v','rtl/common/async_bram_fifo.v')
 tb_video_flow=@('rtl/control/system/video_board_flow.v')
 tb_vision_clock_bridge=@('rtl/memory/ddr/vision_clock_bridge.v','rtl/common/async_fifo.v','rtl/common/async_bram_fifo.v')
 tb_packed_map=@('rtl/compute/geometry/packed_map_builder.v','rtl/image/remap/map_coord_core.v','rtl/compute/float/fp32_service.v','rtl/compute/float/fp_divsqrt.v','rtl/image/remap/sample_coord.v','rtl/image/remap/output_writer.v')
 tb_board_display_switch=@('rtl/video/dma/board_display_hmic.v')
 tb_core_debug=@('rtl/debug/core_debug.v')
 tb_board_debug=@('rtl/debug/board_debug.v','rtl/debug/algorithm_debug.v','rtl/debug/debug_snapshot.v','rtl/common/reset_sync.v','rtl/io/uart/debug_uart.v')
 tb_vision_manual=@('rtl/control/system/vision_sequence.v')
 tb_vision_build_only=@('rtl/control/system/vision_sequence.v','tb/system/tb_vision_manual.sv')
}

$passed=@();$failed=@()
foreach($test in ($tests.Keys | Sort-Object)){
 $benches=@(Get-ChildItem (Join-Path $root 'tb') -Recurse -Filter ($test+'.sv'))
 if($benches.Count -ne 1){throw "Missing/duplicate testbench $test"}
 $srcs=@($tests[$test]|ForEach-Object{Join-Path $root $_})
 if($model){
  $dir=Join-Path $build $test
  New-Item -ItemType Directory -Force $dir | Out-Null
  Push-Location $dir
  try{
   & (Join-Path $ModelSimBin 'vlib.exe') work 2>&1 | Out-File 'vlib.log'
   & (Join-Path $ModelSimBin 'vlog.exe') ('+incdir+'+(Join-Path $root 'rtl/include')) @srcs $benches[0].FullName 2>&1 | Out-File 'vlog.log'
   if($LASTEXITCODE -ne 0){Write-Host "COMPILE FAILED: $test";Get-Content 'vlog.log'|Write-Host;$failed+=$test;continue}
   & (Join-Path $ModelSimBin 'vsim.exe') -c -novopt -l vsim.log -do 'run -all; quit' $test 2>&1 | Out-Null
   $log=Get-Content 'vsim.log' -Raw
  }finally{Pop-Location}
  $ok=($log -match '(?m)^#?\s*PASS') -and ($log -notmatch '(?m)^#?\s*\*\*\s*(Fatal|Error)') -and ($log -match 'Errors: 0')
 }else{
  $output=Join-Path $build ($test+'.vvp')
  Push-Location $root
  try{& (Join-Path $IcarusBin 'iverilog.exe') -g2012 -I rtl/include -s $test -o $output @($tests[$test]) $benches[0].FullName;$code=$LASTEXITCODE}finally{Pop-Location}
  if($code -ne 0){Write-Host "COMPILE FAILED: $test";$failed+=$test;continue}
  Push-Location $build
  try{$log=& (Join-Path $IcarusBin 'vvp.exe') $output 2>&1;$code=$LASTEXITCODE}finally{Pop-Location}
  $log | Out-File (Join-Path $build ($test+'.log'))
  $ok=($code -eq 0) -and (($log -join "`n") -match 'PASS') -and (($log -join "`n") -notmatch '(?im)^(FATAL|ERROR):')
 }
 if($ok){Write-Host "PASS $test";$passed+=$test}
 else{Write-Host "FAIL $test";$failed+=$test}
}
"=== PASS: $($passed.Count) FAIL: $($failed.Count) ==="
if($failed.Count){$failed|ForEach-Object{$_};exit 1}
