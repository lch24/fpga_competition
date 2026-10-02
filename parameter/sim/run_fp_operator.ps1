param([string]$ModelSimBin = 'E:\pangu\Modelsim10.1c\win64')
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    if (!(Test-Path -LiteralPath 'build_fp')) { New-Item -ItemType Directory -Path build_fp | Out-Null }
    Push-Location (Join-Path $PSScriptRoot 'build_fp')
    try {
        if (!(Test-Path -LiteralPath 'work')) {
            & "$ModelSimBin\vlib.exe" work
            if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
        }
        & "$ModelSimBin\vlog.exe" -work work +incdir+../../rtl/common +incdir+../../rtl/math ../../rtl/math/fp_divsqrt.v ../../rtl/math/fp_operator.v
        if ($LASTEXITCODE -ne 0) { throw 'Arithmetic compilation failed' }
        & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../rtl/common ../tb_fp_operator.sv
        if ($LASTEXITCODE -ne 0) { throw 'Testbench compilation failed' }
        & "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_fp_operator -do ../run_fp_operator.do
        if ($LASTEXITCODE -ne 0) { throw 'Floating-point test failed; inspect fp32_results.txt / fp64_results.txt' }
        if (!(Select-String -LiteralPath simulation.log -SimpleMatch 'FP_OPERATOR_PASS' -Quiet)) {
            throw 'Missing pass marker'
        }
        Get-Content fp32_results.txt,fp64_results.txt
    } finally { Pop-Location }
} finally { Pop-Location }
