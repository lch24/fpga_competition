param([string]$ModelSimBin='E:\pangu\Modelsim10.1c\win64',[string]$OnlyTest='')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$build=Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force $build | Out-Null
$rtlRoots=@('algorithom/closer2fpga/rtl','parameter/rtl','undistort/rtl')
$sources=@($rtlRoots | ForEach-Object { Get-ChildItem (Join-Path $root $_) -Recurse -File | Where-Object {$_.Extension -in '.v','.sv'} })
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
$sources=@($sources | Sort-Object @{Expression={if($_.Name -eq 'lround_pkg.sv'){0}else{1}}},FullName)
$tests=@('tb_calibration_mailbox','tb_ddr_adapter','tb_camera_param_store','tb_map_build_ctrl','tb_async_fifo','tb_capture_dma','tb_hdmi_bridge','tb_undistort','tb_camera_system')
$testFiles=@($tests | ForEach-Object {
  # Explicit filename lookup avoids pipeline variable shadowing.
  $sv=Join-Path $root "undistort/tb/$_.sv"
  if(Test-Path $sv){$sv}else{Join-Path $root "undistort/tb/$_.v"}
})
$testFiles+=Join-Path $root 'algorithom/closer2fpga/sim/ddr_memory_model.sv'
$testFiles+=Join-Path $root 'algorithom/closer2fpga/sim/tb_arbiter.sv'
$tests+='tb_arbiter'
$testFiles+=Join-Path $root 'parameter/sim/tb_corner_store.sv'
$testFiles+=Join-Path $root 'parameter/sim/tb_calib_top.sv'
$tests+=@('tb_corner_store','tb_calib_top')
$includes=@('parameter/rtl/common','parameter/rtl/math','undistort/rtl/include') | ForEach-Object {'+incdir+'+(Join-Path $root $_).Replace('\','/')}
$manifest=@($includes)+@($sources.FullName | ForEach-Object {'"'+$_.Replace('\','/')+'"'})+@($testFiles | ForEach-Object {'"'+$_.Replace('\','/')+'"'})
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
  Invoke-Tool 'vlog.exe' @('-sv','-work','work','-f','sources.f') 'compile.log' | Out-Null
  foreach($top in @('calib_top','corner_detect_ddr_top','camera_system_top')) {
    Invoke-Tool 'vopt.exe' @($top,'-o',($top+'_checked')) ($top+'_elaborate.log') | Out-Null
    Write-Host "ELABORATE PASS $top"
  }
  foreach($test in $tests) {
    if($OnlyTest -and $test -ne $OnlyTest){continue}
    # $finish returns to Tcl; a fatal/break or timeout cannot be mistaken for PASS.
    [IO.File]::WriteAllText((Join-Path $build 'run.do'),"onerror {quit -code 1 -f}`nonbreak {quit -code 1 -f}`nrun -all`nquit -code 0 -f`n",[Text.Encoding]::ASCII)
    $simArgs=@('-c',('work.'+$test),'-do','run.do')
    if($test -in @('tb_corner_store','tb_calib_top')) {
      $doName=if($test -eq 'tb_corner_store'){'run_corner_store.do'}else{'run_calib_top.do'}
      $simArgs=@('-c','-voptargs=+acc',('work.'+$test),'-do',(Join-Path $root "parameter/sim/$doName"))
      if($test -eq 'tb_calib_top'){
        $simArgs+='-gCONTROL_ONLY=1'
        $simArgs+='+VECTOR_FILE='+(Join-Path $root 'parameter/sim/calib_top_vectors.txt').Replace('\','/')
      }
    }
    $out=Invoke-Tool 'vsim.exe' $simArgs ($test+'.log')
    if($out -notmatch '(?m)^# (PASS[: ]|TB RESULT: ALL ARBITER TESTS PASSED|CORNER_STORE_PASS |CALIB_TOP_PASS )' -or $out -match '\*\* (Error|Fatal):'){throw "Missing PASS or simulator error in $test"}
    Write-Host "SIM PASS $test"
  }
} finally {Pop-Location}
