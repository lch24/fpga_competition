param([int]$Views=5,[int]$Rows=7,[int]$Cols=10,[int]$MaxIterations=1,[int]$MaxTries=3,[switch]$SkipLm,[switch]$TopOnly,[string]$ModelSimBin=$env:MODELSIM_BIN)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot '../common/tools.ps1')
$ModelSimBin=Resolve-ToolDirectory $ModelSimBin 'vsim.exe' 'MODELSIM_BIN'
$ScriptDir=$PSScriptRoot
$RepoRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$BuildRoot=Join-Path $RepoRoot 'build/calibration'
New-Item -ItemType Directory -Force $BuildRoot | Out-Null

if($Views -lt 3 -or $Views -gt 16 -or $Rows -lt 2 -or $Cols -lt 2 -or $Rows*$Cols -gt 256 -or $MaxIterations -lt 1 -or $MaxIterations -gt 255 -or $MaxTries -lt 1 -or $MaxTries -gt 256) { throw 'Unsupported configuration' }
$suffix=if($TopOnly){'_top'}else{''}
$build=Join-Path $BuildRoot "build_config_${Views}_${Rows}_${Cols}${suffix}"
& node (Join-Path $ScriptDir '../../data/generators/calibration/generate_config_vectors.js') $build $Views $Rows $Cols
if($LASTEXITCODE -ne 0){throw 'Fixture generation failed'}
Push-Location $build
try {
 if(!(Test-Path work)){ & "$ModelSimBin\vlib.exe" work; if($LASTEXITCODE -ne 0){throw 'vlib failed'} }
 $manifest=Get-Content ../../../rtl/files.f | ForEach-Object {if($_ -match '^\+incdir\+'){$_ -replace '^\+incdir\+','+incdir+../../../'}else{'../../../'+$_}}
 Set-Content rtl.f $manifest -Encoding ASCII
 $defines="+define+PAR_VIEWS=$Views+PAR_BOARD_ROWS=$Rows+PAR_BOARD_COLS=$Cols+PAR_LM_MAX_ITERS=$MaxIterations+PAR_LM_MAX_TRIES=$MaxTries"
 & "$ModelSimBin\vlog.exe" -sv -work work $defines -f rtl.f
 if($LASTEXITCODE -ne 0){throw 'RTL compilation failed'}
 $tb=if($TopOnly){'tb_configurable_top'}else{'tb_configurable'}
 & "$ModelSimBin\vlog.exe" -sv -work work $defines +incdir+../../../rtl/include "../../../tb/calibration/$tb.sv"
 if($LASTEXITCODE -ne 0){throw 'TB compilation failed'}
 # Check the real public top can elaborate, including its configuration guard.
 & "$ModelSimBin\vopt.exe" work.calib_top -o checked_top
 if($LASTEXITCODE -ne 0){throw 'Top elaboration failed'}
 $skip=if($SkipLm){1}else{0}
 if($TopOnly) {
  Invoke-Simulator "$ModelSimBin\vsim.exe" -c '-voptargs=+acc=r+/tb_configurable_top -O5' -l simulation.log work.tb_configurable_top -do ../../../scripts/calibration/run_configurable_top.do
 } else {
  Invoke-Simulator "$ModelSimBin\vsim.exe" -c '-voptargs=+acc=r+/tb_configurable -O5' -l simulation.log work.tb_configurable "-gSKIP_LM=$skip" -do ../../../scripts/calibration/run_configurable.do
 }
 if($LASTEXITCODE -ne 0){throw 'Configuration simulation failed'}
 $marker=if($TopOnly){'CONFIGURABLE_TOP_PASS'}else{'CONFIGURABLE_PASS'}
 if(!(Select-String -LiteralPath simulation.log -SimpleMatch $marker -Quiet)){throw 'Missing pass marker'}
 $report=if($TopOnly){'config_top_results.txt'}else{'config_results.txt'}
 Get-Content -LiteralPath $report
} finally {Pop-Location}
