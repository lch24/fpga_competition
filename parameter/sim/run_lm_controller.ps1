param([string]$ModelSimBin='E:\pangu\Modelsim10.1c\win64')
$ErrorActionPreference='Stop'
Push-Location $PSScriptRoot
try {
 if (!(Test-Path -LiteralPath build_lm_controller)) { New-Item -ItemType Directory -Path build_lm_controller | Out-Null }
 Push-Location (Join-Path $PSScriptRoot 'build_lm_controller')
 try {
  if (!(Test-Path -LiteralPath work)) { & "$ModelSimBin\vlib.exe" work; if ($LASTEXITCODE -ne 0) { throw 'vlib failed' } }
  & "$ModelSimBin\vlog.exe" -work work +incdir+../../rtl/common +incdir+../../rtl/math ../../rtl/math/fp_divsqrt.v ../../rtl/math/fp_operator.v ../../rtl/math/gauss_solver.v ../../rtl/model/rotation.v ../../rtl/model/brown_distort.v ../../rtl/model/project_point.v ../../rtl/model/residual_engine.v ../../rtl/calibration/lm/jacobian.v ../../rtl/calibration/lm/normal_equation.v ../../rtl/calibration/lm/damped_step.v ../../rtl/calibration/lm/lm_controller.v
  if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
  & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../rtl/common ../tb_lm_controller.sv
  if ($LASTEXITCODE -ne 0) { throw 'TB compilation failed' }
  & "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_lm_controller -do ../run_lm_controller.do
  if ($LASTEXITCODE -ne 0) { throw 'lm_controller simulation failed' }
  if (!(Select-String -LiteralPath simulation.log -SimpleMatch LM_CONTROLLER_PASS -Quiet)) { throw 'Missing pass marker' }
  Get-Content lm_controller_results.txt
 } finally { Pop-Location }
} finally { Pop-Location }
