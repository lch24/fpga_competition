param([string]$ModelSimBin='E:\pangu\Modelsim10.1c\win64')
$ErrorActionPreference='Stop'
Push-Location $PSScriptRoot
try {
    if (!(Test-Path -LiteralPath build_gauss)) { New-Item -ItemType Directory -Path build_gauss | Out-Null }
    Push-Location (Join-Path $PSScriptRoot 'build_gauss')
    try {
        if (!(Test-Path -LiteralPath work)) {
            & "$ModelSimBin\vlib.exe" work
            if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
        }
        & "$ModelSimBin\vlog.exe" -work work +incdir+../../rtl/common +incdir+../../rtl/math ../../rtl/math/fp_divsqrt.v ../../rtl/math/fp_operator.v ../../rtl/math/gauss_solver.v
        if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
        & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../rtl/common ../tb_gauss_solver.sv
        if ($LASTEXITCODE -ne 0) { throw 'Testbench compilation failed' }
        & "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_gauss_solver -do ../run_gauss_solver.do
        if ($LASTEXITCODE -ne 0) { throw 'Gauss simulation failed; see build_gauss/gauss_results.txt' }
        if (!(Select-String -LiteralPath simulation.log -SimpleMatch GAUSS_SOLVER_PASS -Quiet)) {
            throw 'Missing simulation pass marker'
        }
        Get-Content -LiteralPath gauss_results.txt
    } finally { Pop-Location }
} finally { Pop-Location }
