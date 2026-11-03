param([string]$IcarusBin=$env:IVERILOG_BIN)
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
if(!$IcarusBin){$IcarusBin=Split-Path (Get-Command iverilog.exe -ErrorAction Stop).Source}
$build=Join-Path $root 'build/review'
New-Item -ItemType Directory -Force $build | Out-Null
$tests=[ordered]@{
 tb_corner_review_dma=@('rtl/video/dma/corner_review_dma.v','tb/review/tb_corner_review_dma.sv')
 tb_corner_review_flow=@('rtl/control/system/video_board_flow.v','tb/review/tb_corner_review_flow.sv')
 tb_corner_review_integration=@('rtl/control/system/video_board_flow.v','rtl/control/system/vision_sequence.v','rtl/memory/ddr/vision_clock_bridge.v','rtl/common/async_fifo.v','rtl/video/dma/corner_review_dma.v','tb/review/tb_corner_review_integration.sv')
 tb_corner_review_fast_algorithm=@('rtl/control/system/video_board_flow.v','rtl/control/system/vision_sequence.v','rtl/memory/ddr/vision_clock_bridge.v','rtl/common/async_fifo.v','rtl/video/dma/corner_review_dma.v','tb/review/tb_corner_review_integration.sv','tb/review/tb_corner_review_fast_algorithm.sv')
 tb_corner_review_native=@('rtl/control/system/video_board_flow.v','rtl/control/system/vision_sequence.v','rtl/memory/ddr/vision_clock_bridge.v','rtl/common/async_fifo.v','rtl/video/dma/corner_review_dma.v','rtl/image/remap/realtime_remap.v','rtl/image/remap/video_map_prefetch.v','rtl/image/remap/video_grid_prefetch.v','rtl/image/remap/video_neighbor_cache.v','rtl/image/kernels/video_bilinear_pipe.v','rtl/video/dma/video_frame_writer.v','rtl/memory/ddr/video_hmic_arbiter.v','tb/review/tb_corner_review_integration.sv','tb/review/tb_corner_review_native.sv')
 tb_corner_review_sequence=@('rtl/control/system/vision_sequence.v','tb/system/tb_vision_manual.sv','tb/review/tb_corner_review_sequence.sv')
 tb_vision_manual=@('rtl/control/system/vision_sequence.v','tb/system/tb_vision_manual.sv')
 tb_video_flow=@('rtl/control/system/video_board_flow.v','tb/system/tb_video_flow.sv')
 tb_vision_clock_bridge=@('rtl/memory/ddr/vision_clock_bridge.v','rtl/common/async_fifo.v','rtl/common/async_bram_fifo.v','tb/memory/tb_vision_clock_bridge.sv')
}
$results=@()
Push-Location $root
try {
 foreach($name in $tests.Keys){
  $target=Join-Path $build ($name+'.vvp')
  & (Join-Path $IcarusBin 'iverilog.exe') -g2012 -I rtl/include -s $name -o $target @($tests[$name])
  if($LASTEXITCODE -ne 0){throw "Compile failed: $name"}
  $output=& (Join-Path $IcarusBin 'vvp.exe') $target 2>&1
  $code=$LASTEXITCODE;$text=$output -join "`n"
  $output | Out-File (Join-Path $build ($name+'.log')) -Encoding utf8
  if($code -ne 0 -or $text -notmatch '(?m)^PASS' -or $text -match '(?im)^(FATAL|ERROR):'){throw "Failed: $name`n$text"}
  $results+=@{test=$name;status='PASS'};Write-Host "PASS $name"
 }
 & python scripts/debug/uart_monitor.py --self-test
 if($LASTEXITCODE -ne 0){throw 'Host decoder failed'}
 $results | ConvertTo-Json | Set-Content (Join-Path $build 'test_results.json') -Encoding utf8
}finally{Pop-Location}
Write-Host 'PASS corner review checks; no EDA or hardware access'
