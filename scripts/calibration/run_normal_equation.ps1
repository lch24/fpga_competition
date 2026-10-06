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
 if (!(Test-Path -LiteralPath build_normal_equation)) { New-Item -ItemType Directory -Path build_normal_equation | Out-Null }
 Push-Location (Join-Path $BuildRoot 'build_normal_equation')
 try {
  if (!(Test-Path -LiteralPath work)) { & "$ModelSimBin\vlib.exe" work; if ($LASTEXITCODE -ne 0) { throw 'vlib failed' } }
  & "$ModelSimBin\vlog.exe" -work work +incdir+../../../rtl/include +incdir+../../../rtl/compute/float ../../../rtl/compute/float/fp_divsqrt.v ../../../rtl/compute/float/fp_operator.v ../../../rtl/compute/service/fp_calibration_pool.v ../../../rtl/compute/linalg/gauss_solver.v ../../../rtl/compute/geometry/rotation.v ../../../rtl/memory/local/work_ram.v ../../../rtl/compute/geometry/geometry_engine.v ../../../rtl/compute/geometry/brown_distort.v ../../../rtl/compute/geometry/project_point.v ../../../rtl/compute/geometry/residual_engine.v ../../../rtl/compute/service/residual_endpoint.v ../../../rtl/control/calibration/lm/jacobian.v ../../../rtl/control/calibration/lm/normal_equation.v ../../../rtl/control/calibration/lm/damped_step.v ../../../rtl/control/calibration/lm/lm_controller.v
  if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
  & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../../rtl/include ../../../tb/calibration/tb_normal_equation.sv
  if ($LASTEXITCODE -ne 0) { throw 'TB compilation failed' }
  Invoke-Simulator "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_normal_equation -do ../../../scripts/calibration/run_normal_equation.do
  if ($LASTEXITCODE -ne 0) { throw 'normal_equation simulation failed' }
  if (!(Select-String -LiteralPath simulation.log -SimpleMatch NORMAL_EQUATION_PASS -Quiet)) { throw 'Missing pass marker' }
  Get-Content normal_equation_results.txt
 } finally { Pop-Location }
} finally { Pop-Location }
