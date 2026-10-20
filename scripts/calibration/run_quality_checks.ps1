param([string]$IcarusBin=$env:IVERILOG_BIN)
# Targeted policy/control/UART regression, not a full numerical/physical signoff.
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$out=Join-Path $root 'build/quality'
New-Item -ItemType Directory -Force $out | Out-Null
if(!$IcarusBin){$tool=Get-Command iverilog.exe -ErrorAction SilentlyContinue;if($tool){$IcarusBin=Split-Path $tool.Source}}
if(!$IcarusBin){throw 'Pass -IcarusBin with the directory containing iverilog.exe and vvp.exe'}
Push-Location $root
try {
 & python scripts/calibration/engine/build_rom.py
 if($LASTEXITCODE){throw 'ROM build failed'}
 & python scripts/debug/check_calibration_quality.py
 if($LASTEXITCODE){throw 'Coverage numerical/instruction checks failed'}
 # Do not parse unrelated detection sources: portable Icarus 10 does not support
 # their generate syntax. Actual calibration dependencies remain real RTL.
 $sources=@(Get-ChildItem rtl/control/calibration,rtl/compute,rtl/memory/parameters,rtl/memory/local,rtl/common -Recurse -File |
   Where-Object {$_.Extension -in '.v','.sv'} | ForEach-Object {$_.FullName})
 & "$IcarusBin/iverilog.exe" -g2012 -I rtl/include -s tb_calib_top -P tb_calib_top.CONTROL_ONLY=1 -P tb_calib_top.FINISH_ON_DONE=1 -o "$out/calib_control.vvp" @sources tb/calibration/tb_calib_top.sv
 if($LASTEXITCODE){throw 'Control TB compile failed'}
 & "$IcarusBin/iverilog.exe" -g2012 -I rtl/include -s tb_init_coverage_transport -o "$out/init_transport.vvp" @sources tb/calibration/tb_init_coverage_transport.sv
 if($LASTEXITCODE){throw 'Initialization transport TB compile failed'}
 & "$IcarusBin/iverilog.exe" -g2012 -I rtl/include -s tb_board_debug -o "$out/uart.vvp" rtl/debug/board_debug.v rtl/debug/algorithm_debug.v rtl/debug/debug_snapshot.v rtl/common/reset_sync.v rtl/io/uart/debug_uart.v tb/debug/tb_board_debug.sv
 if($LASTEXITCODE){throw 'UART TB compile failed'}
 Push-Location $out
 try {
  foreach($test in @('calib_control','init_transport','uart')){
   $vectorPath=(Join-Path $root 'data/calibration/calib_top_vectors.txt').Replace('\','/')
   $extra=@(if($test -eq 'calib_control'){"+VECTOR_FILE=$vectorPath"})
   $log=& "$IcarusBin/vvp.exe" "$out/$test.vvp" @extra 2>&1
   $code=$LASTEXITCODE
   $log | Out-File "$test.log"
   $text=$log -join "`n"
   if($code -or $text -notmatch '(?m)^PASS' -or $text -match '(?im)^(FATAL|ERROR):'){throw "$test failed: $text"}
   Write-Output $text
  }
 } finally {Pop-Location}
 & python tb/debug/check_uart_trace.py "$out/uart_packets.hex"
 if($LASTEXITCODE){throw 'UART host cross-check failed'}
 Write-Output 'PASS calibration quality directed regression'
} finally {Pop-Location}
