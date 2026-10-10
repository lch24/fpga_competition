param([string]$ModelSimBin=$env:MODELSIM_BIN,[string]$OnlyTest='', [string]$Configuration='', [switch]$Board, [string]$PythonBin='python')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot '../common/tools.ps1')
$ModelSimBin=Resolve-ToolDirectory $ModelSimBin 'vsim.exe' 'MODELSIM_BIN'
if($OnlyTest -in @('tb_reg_config_clock','tb_algorithm_clock') -and !$Board){throw 'tb_reg_config_clock requires -Board'}
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$build=Join-Path $root 'build/system'
$defines=@()
if($Configuration){
  if($OnlyTest -ne 'tb_vision_ddr'){throw 'Configuration override currently supports tb_vision_ddr only'}
  foreach($entry in $Configuration.Split(',')){
    if($entry -notmatch '^PAR_(VIEWS|BOARD_ROWS|BOARD_COLS|LM_MAX_ITERS|LM_MAX_TRIES)=[0-9]+$'){throw "Invalid configuration: $entry"}
    $defines+='+define+'+$entry
  }
  $build=Join-Path $build ('config_'+$Configuration.Replace(',','_').Replace('=','_'))
}
New-Item -ItemType Directory -Force $build | Out-Null
if(!$OnlyTest -or $OnlyTest -eq 'tb_fp64_sqrt_serial'){
  & $PythonBin (Join-Path $root 'data/generators/system/generate_sqrt_vectors.py')
  if($LASTEXITCODE -ne 0){throw 'Could not generate sqrt vectors'}
}
New-Item -ItemType Directory -Force (Join-Path $build 'data/rom') | Out-Null
Copy-Item (Join-Path $root 'data/rom/*.mem') (Join-Path $build 'data/rom') -Force
if(!$OnlyTest -or $OnlyTest -in @('tb_detection_compat','tb_detection_fixed','tb_vision_numeric')){
  & node (Join-Path $root 'data/generators/system/generate_board_images.js') | Out-Null
  if($LASTEXITCODE -ne 0){throw 'Could not generate independent detection images'}
}
$sources=@(Get-Content (Join-Path $root 'rtl/files.f') | Where-Object {$_ -and !$_.StartsWith('+')} | ForEach-Object {Get-Item (Join-Path $root $_)})
# Package must precede ring_check. Reject duplicate modules before compiling.
$names=@{}
foreach($file in $sources) {
  $text=[IO.File]::ReadAllText($file.FullName)
  $text=[regex]::Replace($text,'(?s)/\*.*?\*/|//[^\r\n]*','')
  foreach($m in [regex]::Matches($text,'\bmodule\s+(\w+)')) {
    $name=$m.Groups[1].Value
    if($names.ContainsKey($name)){throw "Duplicate module $name : $($names[$name]) and $($file.FullName)"}
    $names[$name]=$file.FullName
  }
}
$sources=@($sources | Sort-Object FullName)
if($Board){
  # Vendor IP port-only stubs are for connection elaboration, not IP simulation.
  $stub=''
  foreach($ip in @('pll/pll.v','ddr3_test/ddr3_test.v')){
    $source=[IO.File]::ReadAllText((Join-Path $root ('OV5640_DualView_100H/ipcore/'+$ip)))
    $first=[regex]::Match($source,'\bmodule\s+').Index
    $last=$source.IndexOf(');',$first)+2
    $stub+=$source.Substring($first,$last-$first)+"`nendmodule`n"
  }
  $stub+='module GTP_INBUFGDS #(parameter IOSTANDARD="DEFAULT",TERM_DIFF="ON")(output O,input I,IB); assign O=I; endmodule'
  $stubPath=Join-Path $build 'board_ip_stubs.v'
  [IO.File]::WriteAllText($stubPath,$stub,[Text.Encoding]::ASCII)
  $legacyBoardFiles=@()
  foreach($name in @('sync_vg','ms7200_ctl','ms7210_ctl','iic_dri','i2c_com','reg_config')){
    $legacyBoardFiles+=Join-Path $root ('rtl/video/board/'+$name+'.v')
  }
  $sources+=Get-Item (Join-Path $root 'rtl/top/calibrated_view_top.v')
  $sources+=Get-Item (Join-Path $root 'rtl/clock/algorithm_clock.v')
  $pdsHome=if($env:PDS_HOME){$env:PDS_HOME}else{'D:/pango/PDS_2025.2-ads'}
  $sources+=Get-Item (Join-Path $pdsHome 'arch/vendor/pango/verilog/simulation/GTP_GPLL.v')
  $sources+=Get-Item (Join-Path $root 'rtl/video/board/board_ms72xx_ctl.v')
  $sources+=Get-Item (Join-Path $root 'rtl/video/board/board_power_on_delay.v')
  $sources+=Get-Item $stubPath
}
$tests=@('tb_calibration_mailbox','tb_ddr_adapter','tb_camera_param_store','tb_map_build_ctrl','tb_async_fifo','tb_capture_dma','tb_hdmi_bridge','tb_undistort','tb_camera_system')
function Find-TestFile([string]$name) {
  $found=@(Get-ChildItem (Join-Path $root 'tb') -Recurse -File | Where-Object {$_.BaseName -eq $name -and $_.Extension -in @('.v','.sv')})
  if($found.Count -ne 1){throw "Expected one testbench for $name, found $($found.Count)"}
  return $found[0].FullName
}
function Find-DoFile([string]$name) {
  $found=@(Get-ChildItem (Join-Path $root 'scripts') -Recurse -Filter $name)
  if($found.Count -ne 1){throw "Expected one Tcl driver for $name"}
  return 'do {'+$found[0].FullName.Replace('\','/')+'}'
}
$testFiles=@($tests | ForEach-Object {Find-TestFile $_})
$testFiles+=Join-Path $root 'tb/models/ddr_memory_model.sv'
$testFiles+=Join-Path $root 'tb/memory/tb_arbiter.sv'
$tests+='tb_arbiter'
$testFiles+=Join-Path $root 'tb/memory/tb_adapter.sv'
$tests+='tb_adapter'
$testFiles+=Join-Path $root 'tb/memory/tb_corner_store.sv'
$testFiles+=Join-Path $root 'tb/calibration/tb_calib_top.sv'
$tests+=@('tb_corner_store','tb_calib_top')
$integrationPaths=Get-Content (Join-Path $PSScriptRoot 'integration_tests.json') -Raw | ConvertFrom-Json
$integrationTests=@(($integrationPaths | ForEach-Object {Get-Item (Join-Path $root $_)}) | Where-Object {
  ($Board -or $_.BaseName -ne 'tb_algorithm_clock') -and $_.BaseName -notin @('tb_detection_program','tb_detection_flow_control','tb_fp32_pair_add_pool','tb_feature_program','tb_accum_fixed','tb_add_pipeline','tb_tensor_shared','tb_bilinear_fixed','tb_resource_math','tb_gray_cache','tb_fp_pool','tb_ddr_pyramid','tb_ddr_recovery','tb_calib_geometry')
}) # These tests have independent vector generation in tools/check_resource_units.py.
$testFiles+=@($integrationTests.FullName)
$tests+=@($integrationTests.BaseName)
if($OnlyTest -and $OnlyTest -notin $tests){throw "Unknown test: $OnlyTest"}
$includes=@('rtl/include','rtl/compute/float','rtl/include','tb/models') | ForEach-Object {'"+incdir+'+(Join-Path $root $_).Replace('\','/')+'"'}
$manifest=@($defines)+@($includes)+@($sources.FullName | ForEach-Object {'"'+$_.Replace('\','/')+'"'})+@($testFiles | ForEach-Object {'"'+$_.Replace('\','/')+'"'})
[IO.File]::WriteAllLines((Join-Path $build 'sources.f'),$manifest,[Text.Encoding]::ASCII)
function Invoke-Tool([string]$tool,[string[]]$arguments,[string]$log) {
  # ModelSim 10.1c can print a harmless FileWatch Tcl error while exiting.
  $savedPreference=$ErrorActionPreference
  $ErrorActionPreference='Continue'
  $output=& (Join-Path $ModelSimBin $tool) @arguments 2>&1
  $exitCode=$LASTEXITCODE
  $ErrorActionPreference=$savedPreference
  $output | Out-File (Join-Path $build $log)
  if($exitCode -ne 0){throw "$tool failed ($exitCode); see $build/$log"}
  return ($output -join "`n")
}
Push-Location $build
try {
  if(!(Test-Path work)){Invoke-Tool 'vlib.exe' @('work') 'vlib.log' | Out-Null}
  if($Board){Invoke-Tool 'vlog.exe' (@('-work','work')+$legacyBoardFiles) 'board_legacy_compile.log' | Out-Null}
  Invoke-Tool 'vlog.exe' @('-sv','-work','work','-f','sources.f') 'compile.log' | Out-Null
  foreach($top in @('calib_top','corner_detect_ddr_top','camera_system_top','vision_ddr_top','vision_camera_top')) {
    Invoke-Tool 'vopt.exe' @($top,'-o',($top+'_checked')) ($top+'_elaborate.log') | Out-Null
    Write-Host "ELABORATE PASS $top"
  }
  if($Board){
    Invoke-Tool 'vopt.exe' @('calibrated_view_top','-o','calibrated_view_top_checked') 'board_elaborate.log' | Out-Null
    Write-Host 'ELABORATE PASS calibrated_view_top (vendor IP port stubs only)'
  }
  foreach($test in $tests) {
    if($OnlyTest -and $test -ne $OnlyTest){continue}
    if($test -eq 'tb_reg_config_clock' -and !$Board){continue}
    if(!$OnlyTest -and $test -eq 'tb_vision_numeric'){continue} # opt-in long numerical run
    # $finish returns to Tcl; a fatal/break or timeout cannot be mistaken for PASS.
    [IO.File]::WriteAllText((Join-Path $build 'run.do'),"onerror {quit -code 1 -f}`nonbreak {quit -code 1 -f}`nrun -all`nquit -code 0 -f`n",[Text.Encoding]::ASCII)
    $simArgs=@('-c',('work.'+$test),'-do','run.do')
    # Check completion counters in Tcl as well as HDL diagnostics on ModelSim 10.1c.
    if($test -in @('tb_detection_capture','tb_board_flow_cdc','tb_algorithm_clock','tb_vision_clock_bridge','tb_board_flow','tb_detection_fixed','tb_grid_shared','tb_subpixel_patch','tb_candidate_recovery','tb_merge_bitmap','tb_vision_ddr','tb_candidate_cache','tb_undistort','tb_vision_numeric','tb_fp64_add_bounds','tb_detection_compat','tb_reg_config_clock','tb_fp64_sqrt_serial','tb_grid_validate_serial','tb_response_replay','tb_replay_pyramid')) {
      $doName=$test.Replace('tb_','run_')+'.do'
      $simArgs=@('-c','-voptargs=+acc',('work.'+$test),'-do',(Find-DoFile $doName))
      if($test -eq 'tb_vision_numeric'){$simArgs=@('-c','-voptargs=+acc=rn+tb_vision_numeric -O5',('work.'+$test),'-do',(Find-DoFile $doName))}
      if($test -in @('tb_detection_compat','tb_detection_fixed')){
        # Only the scoreboard needs Tcl visibility; keep arithmetic optimizable.
        $simArgs=@('-c','-voptargs=+acc=rn+tb_vision_numeric',('work.'+$test),'-do',(Find-DoFile $doName))
      }
    }
    if($test -in @('tb_corner_store','tb_calib_top')) {
      $doName=if($test -eq 'tb_corner_store'){'run_corner_store.do'}else{'run_calib_top.do'}
      $simArgs=@('-c','-voptargs=+acc',('work.'+$test),'-do',(Find-DoFile $doName))
      if($test -eq 'tb_calib_top'){
        $simArgs+='-gCONTROL_ONLY=1'
        $simArgs+='+VECTOR_FILE='+(Join-Path $root 'data/calibration/calib_top_vectors.txt').Replace('\','/')
      }
    }
    $out=Invoke-Tool 'vsim.exe' $simArgs ($test+'.log')
    if($out -notmatch '(?m)^# (PASS[: ]|TB RESULT: ALL ((ARBITER|ADAPTER) )?TESTS PASSED|CORNER_STORE_PASS |CALIB_TOP_PASS )' -or $out -match '\*\* (Error|Fatal):'){throw "Missing PASS or simulator error in $test"}
    Write-Host "SIM PASS $test"
  }
} finally {Pop-Location}
