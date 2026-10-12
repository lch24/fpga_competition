param([string]$IcarusBin=$env:IVERILOG_BIN)
$ErrorActionPreference='Stop'
if(!$IcarusBin){$IcarusBin=Split-Path (Get-Command iverilog.exe -ErrorAction Stop).Source}
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$build=Join-Path $root 'build/realtime'
New-Item -ItemType Directory -Force $build | Out-Null
$tests=@{
 tb_realtime_remap=@('rtl/image/remap/realtime_remap.v','rtl/image/remap/video_map_prefetch.v','rtl/image/remap/video_neighbor_cache.v','rtl/image/kernels/video_bilinear_pipe.v','rtl/video/dma/video_frame_writer.v','rtl/memory/ddr/video_hmic_arbiter.v')
 tb_video_capture=@('rtl/video/dma/video_capture_hmic.v','rtl/common/async_fifo.v','rtl/common/async_bram_fifo.v')
 tb_video_flow=@('rtl/control/system/video_board_flow.v')
 tb_packed_map=@('rtl/compute/geometry/packed_map_builder.v','rtl/image/remap/map_coord_core.v','rtl/compute/float/fp32_service.v','rtl/compute/float/fp_divsqrt.v','rtl/image/remap/sample_coord.v','rtl/image/remap/output_writer.v')
 tb_board_display_switch=@('rtl/video/dma/board_display_hmic.v')
 tb_core_debug=@('rtl/debug/core_debug.v')
 tb_board_debug=@('rtl/debug/board_debug.v','rtl/debug/algorithm_debug.v','rtl/debug/debug_snapshot.v','rtl/common/reset_sync.v','rtl/io/uart/debug_uart.v')
 tb_vision_manual=@('rtl/control/system/vision_sequence.v')
 tb_vision_build_only=@('rtl/control/system/vision_sequence.v','tb/system/tb_vision_manual.sv')
}
Push-Location $root
try{
 foreach($test in ($tests.Keys | Sort-Object)){
  $output=Join-Path $build ($test+'.vvp')
  $benches=@(Get-ChildItem (Join-Path $root 'tb') -Recurse -Filter ($test+'.sv'))
  if($benches.Count -ne 1){throw "Missing/duplicate testbench $test"}
  & (Join-Path $IcarusBin 'iverilog.exe') -g2012 -I rtl/include -s $test -o $output @($tests[$test]) $benches[0].FullName
  if($LASTEXITCODE -ne 0){throw "Compile failed: $test"}
  Push-Location $build
  try{$log=& (Join-Path $IcarusBin 'vvp.exe') $output 2>&1;$code=$LASTEXITCODE}finally{Pop-Location}
  $log | Out-File (Join-Path $build ($test+'.log'))
  $log | Write-Host
  if($code -ne 0 -or ($log -join "`n") -notmatch 'PASS' -or ($log -join "`n") -match '(?im)^(FATAL|ERROR):'){throw "Simulation failed: $test"}
 }
}finally{Pop-Location}
