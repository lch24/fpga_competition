param([string]$ModelSimBin=$env:MODELSIM_BIN)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot '../common/tools.ps1')
$ModelSimBin=Resolve-ToolDirectory $ModelSimBin 'vsim.exe' 'MODELSIM_BIN'
$ScriptDir=$PSScriptRoot
$RepoRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$BuildRoot=Join-Path $RepoRoot 'build/calibration'
New-Item -ItemType Directory -Force $BuildRoot | Out-Null

Push-Location $BuildRoot
try {
 if (!(Test-Path -LiteralPath build_lm_controller)) { New-Item -ItemType Directory -Path build_lm_controller | Out-Null }
 Push-Location (Join-Path $BuildRoot 'build_lm_controller')
 try {
  if (!(Test-Path -LiteralPath work)) { & "$ModelSimBin\vlib.exe" work; if ($LASTEXITCODE -ne 0) { throw 'vlib failed' } }
  & "$ModelSimBin\vlog.exe" -work work +incdir+../../../rtl/include +incdir+../../../rtl/compute/float ../../../rtl/compute/float/fp_divsqrt.v ../../../rtl/compute/float/fp_operator.v ../../../rtl/compute/service/fp_calibration_pool.v ../../../rtl/compute/linalg/gauss_solver.v ../../../rtl/compute/geometry/rotation.v ../../../rtl/memory/local/work_ram.v ../../../rtl/compute/geometry/geometry_engine.v ../../../rtl/compute/geometry/brown_distort.v ../../../rtl/compute/geometry/project_point.v ../../../rtl/compute/geometry/residual_engine.v ../../../rtl/compute/service/residual_endpoint.v ../../../rtl/control/calibration/lm/jacobian.v ../../../rtl/control/calibration/lm/normal_equation.v ../../../rtl/control/calibration/lm/damped_step.v ../../../rtl/control/calibration/lm/lm_controller.v
  if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
  & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../../rtl/include ../../../tb/calibration/tb_lm_controller.sv
  if ($LASTEXITCODE -ne 0) { throw 'TB compilation failed' }
  Invoke-Simulator "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_lm_controller -do ../../../scripts/calibration/run_lm_controller.do
  if ($LASTEXITCODE -ne 0) { throw 'lm_controller simulation failed' }
  if (!(Select-String -LiteralPath simulation.log -SimpleMatch LM_CONTROLLER_PASS -Quiet)) { throw 'Missing pass marker' }
  Get-Content lm_controller_results.txt
 } finally { Pop-Location }
} finally { Pop-Location }
