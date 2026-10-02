param([string]$ModelSimBin='E:\pangu\Modelsim10.1c\win64')
$ErrorActionPreference='Stop'
Push-Location $PSScriptRoot
try {
 if (!(Test-Path -LiteralPath build_validate_result)) { New-Item -ItemType Directory -Path build_validate_result | Out-Null }
 Push-Location (Join-Path $PSScriptRoot 'build_validate_result')
 try {
  if (!(Test-Path -LiteralPath work)) { & "$ModelSimBin\vlib.exe" work; if ($LASTEXITCODE -ne 0) { throw 'vlib failed' } }
  & "$ModelSimBin\vlog.exe" -work work +incdir+../../rtl/common +incdir+../../rtl/math ../../rtl/math/fp_divsqrt.v ../../rtl/math/fp_operator.v ../../rtl/model/rotation.v ../../rtl/model/brown_distort.v ../../rtl/model/project_point.v ../../rtl/model/residual_engine.v ../../rtl/calibration/check/validate_result.v
  if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
  & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../rtl/common ../tb_validate_result.sv
  if ($LASTEXITCODE -ne 0) { throw 'TB compilation failed' }
  & "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_validate_result -do ../run_validate_result.do
  if ($LASTEXITCODE -ne 0) { throw 'validate_result simulation failed' }
  if (!(Select-String -LiteralPath simulation.log -SimpleMatch VALIDATE_RESULT_PASS -Quiet)) { throw 'Missing pass marker' }
  Get-Content validate_result_results.txt
 } finally { Pop-Location }
} finally { Pop-Location }
