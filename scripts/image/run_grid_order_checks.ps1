param([string]$ModelSimBin=$env:MODELSIM_BIN)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot '../common/tools.ps1')
$ModelSimBin=Resolve-ToolDirectory $ModelSimBin 'vsim.exe' 'MODELSIM_BIN'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$build=Join-Path $root 'build/grid_order'
New-Item -ItemType Directory -Force $build | Out-Null
Push-Location $root
try {
 $library=Join-Path $build 'work'
 if(!(Test-Path $library)){& (Join-Path $ModelSimBin 'vlib.exe') $library}
 & (Join-Path $ModelSimBin 'vlog.exe') -sv -work $library -f rtl/files.f tb/image/tb_grid_order_robust.sv *> (Join-Path $build 'compile.log')
 if($LASTEXITCODE -ne 0){throw 'Compile failed: build/grid_order/compile.log'}
 foreach($shared in @(0,1)) {
  $log=Join-Path $build "shared_$shared.log"
  Invoke-Simulator (Join-Path $ModelSimBin 'vsim.exe') -c -novopt -lib $library "-gSHARED=$shared" "-gEXTERNAL=$shared" -l $log tb_grid_order_robust -do scripts/image/run_grid_order_robust.do | Out-File (Join-Path $build "shared_$shared.console.log")
  if($LASTEXITCODE -ne 0 -or (Get-Content $log -Raw) -notmatch '# PASS grid order robust:') {throw "FAIL SHARED=$shared; see $log"}
  Write-Host "PASS grid order: 8 cases, SHARED=$shared"
 }
} finally {Pop-Location}
