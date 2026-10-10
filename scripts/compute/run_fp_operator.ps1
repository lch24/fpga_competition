param([string]$ModelSimBin = $env:MODELSIM_BIN)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot '../common/tools.ps1')
$ModelSimBin=Resolve-ToolDirectory $ModelSimBin 'vsim.exe' 'MODELSIM_BIN'
$ScriptDir=$PSScriptRoot
$RepoRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$BuildRoot=Join-Path $RepoRoot 'build/compute'
New-Item -ItemType Directory -Force $BuildRoot | Out-Null

Push-Location $BuildRoot
try {
    if (!(Test-Path -LiteralPath 'build_fp')) { New-Item -ItemType Directory -Path build_fp | Out-Null }
    Push-Location (Join-Path $BuildRoot 'build_fp')
    try {
        if (!(Test-Path -LiteralPath 'work')) {
            & "$ModelSimBin\vlib.exe" work
            if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
        }
        & "$ModelSimBin\vlog.exe" -work work +incdir+../../../rtl/include +incdir+../../../rtl/compute/float ../../../rtl/compute/float/fp_divsqrt.v ../../../rtl/compute/float/calib_alu.v ../../../rtl/compute/float/fp_math_program.v ../../../rtl/compute/float/fp_operator.v ../../../rtl/compute/service/fp_calibration_pool.v
        if ($LASTEXITCODE -ne 0) { throw 'Arithmetic compilation failed' }
        & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../../rtl/include ../../../tb/compute/tb_fp_operator.sv
        if ($LASTEXITCODE -ne 0) { throw 'Testbench compilation failed' }
        Invoke-Simulator "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_fp_operator -do ../../../scripts/compute/run_fp_operator.do
        if ($LASTEXITCODE -ne 0) { throw 'Floating-point test failed; inspect fp32_results.txt / fp64_results.txt' }
        if (!(Select-String -LiteralPath simulation.log -SimpleMatch 'FP_OPERATOR_PASS' -Quiet)) {
            throw 'Missing pass marker'
        }
        Get-Content fp32_results.txt,fp64_results.txt
    } finally { Pop-Location }
} finally { Pop-Location }
