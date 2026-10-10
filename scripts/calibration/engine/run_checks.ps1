param([string]$ModelSimBin=$env:MODELSIM_BIN,[string]$Python='python')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot '../../common/tools.ps1')
$ModelSimBin=Resolve-ToolDirectory $ModelSimBin 'vsim.exe' 'MODELSIM_BIN'
$RepoRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../..'))
& $Python (Join-Path $PSScriptRoot 'generate_checks.py')
if($LASTEXITCODE){throw 'test generation failed'}
& $Python (Join-Path $PSScriptRoot 'reference.py')
if($LASTEXITCODE){throw 'instruction reference failed'}
Push-Location (Join-Path $RepoRoot 'build/calibration_engine')
try {
 if(!(Test-Path work)){ & "$ModelSimBin/vlib.exe" work; if($LASTEXITCODE){throw 'vlib failed'} }
 & "$ModelSimBin/vlog.exe" -sv +incdir+../../rtl/include ../../rtl/compute/float/fp_divsqrt.v ../../rtl/compute/float/calib_alu.v ../../rtl/control/calibration/engine/calib_sequencer.v ../../rtl/control/calibration/engine/calib_kernel_ctrl.v ../../rtl/memory/local/calib_workspace.v ../../rtl/compute/service/calib_datapath.v ../../tb/control/calibration/engine/tb_calib_engine.sv
 if($LASTEXITCODE){throw 'compile failed'}
 Invoke-Simulator "$ModelSimBin/vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_calib_engine -do ../../scripts/calibration/engine/run_checks.do
 if($LASTEXITCODE){throw 'engine simulation failed'}
 if(!(Select-String -LiteralPath simulation.log -SimpleMatch -Pattern CALIB_ENGINE_PASS -Quiet)){throw 'missing pass marker'}
 & $Python (Join-Path $PSScriptRoot 'check_results.py')
 if($LASTEXITCODE){throw 'instruction census mismatch'}
} finally {Pop-Location}
