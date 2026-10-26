param([string]$IcarusBin=$env:IVERILOG_BIN,[switch]$Quick)
# Isolated, small RTL simulations. No PDS/synthesis/serial/hardware calls.
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
if(!$IcarusBin){$IcarusBin=Split-Path (Get-Command iverilog.exe -ErrorAction Stop).Source}
$iv=Join-Path $IcarusBin 'iverilog.exe';$vvp=Join-Path $IcarusBin 'vvp.exe'
$build=Join-Path $root 'build/advanced'
New-Item -ItemType Directory -Force $build | Out-Null
$remap=@('rtl/image/remap/realtime_remap.v','rtl/image/remap/video_map_prefetch.v','rtl/image/remap/video_neighbor_cache.v','rtl/image/remap/video_affine_coords.v','rtl/image/kernels/video_bilinear_pipe.v','rtl/image/kernels/video_adaptive_gamma.v','rtl/video/dma/video_frame_writer.v','rtl/memory/ddr/video_hmic_arbiter.v','tb/system/tb_realtime_remap.sv')
$debug=@('rtl/debug/board_debug.v','rtl/debug/debug_snapshot.v','rtl/common/reset_sync.v','rtl/io/uart/debug_uart.v')
$tests=[ordered]@{
 tb_affine_coords=@('rtl/image/remap/video_affine_coords.v','tb/advanced/tb_affine_coords.sv')
 tb_adaptive_gamma=@('rtl/image/kernels/video_adaptive_gamma.v','tb/advanced/tb_adaptive_gamma.sv')
 tb_advanced_control=@('rtl/control/system/advanced_config_bridge.v','rtl/control/system/advanced_frame_scheduler.v','tb/advanced/tb_advanced_control.sv')
 tb_advanced_uart=$debug+@('rtl/control/system/advanced_config_bridge.v','tb/advanced/tb_advanced_uart.sv')
 tb_board_debug=$debug+@('rtl/debug/algorithm_debug.v','tb/debug/tb_board_debug.sv')
}
if(!$Quick){
 $tests['tb_realtime_remap']=$remap
 $tests['tb_advanced_remap']=$remap+@('tb/advanced/tb_advanced_remap.sv')
}
$results=@()
Push-Location $root
try {
 foreach($name in $tests.Keys){
  Write-Host "Running $name"
  $target=Join-Path $build ($name+'.vvp')
  & $iv -g2012 -I rtl/include -s $name -o $target @($tests[$name])
  if($LASTEXITCODE -ne 0){throw "Compile failed: $name"}
  Push-Location $build
  try {$log=& $vvp $target 2>&1;$code=$LASTEXITCODE}finally{Pop-Location}
  $log | Out-File (Join-Path $build ($name+'.log')) -Encoding utf8
  $text=$log -join "`n"
  if($code -ne 0 -or $text -notmatch '(?m)^PASS' -or $text -match '(?im)^(FATAL|ERROR):'){throw "Simulation failed: $name`n$text"}
  $results+=@{test=$name;status='PASS'}
  Write-Host "PASS $name"
 }
 & python scripts/debug/test_advanced_control.py
 if($LASTEXITCODE -ne 0){throw 'Host protocol tests failed'}
 $results+=@{test='host_protocol_4_tests';status='PASS'}
 $reportName=if($Quick){'test_results_quick.json'}else{'test_results.json'}
 $results | ConvertTo-Json | Set-Content (Join-Path $build $reportName) -Encoding utf8
}finally{Pop-Location}
Write-Host "PASS advanced checks ($($results.Count) groups), quick=$Quick; no hardware accessed"
