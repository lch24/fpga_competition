param([string]$ModelSimBin='E:\pangu\Modelsim10.1c\win64')
$ErrorActionPreference='Stop'
Push-Location $PSScriptRoot
try {
    if (!(Test-Path -LiteralPath build_eigen)) { New-Item -ItemType Directory -Path build_eigen | Out-Null }
    Push-Location (Join-Path $PSScriptRoot 'build_eigen')
    try {
        if (!(Test-Path -LiteralPath work)) {
            & "$ModelSimBin\vlib.exe" work
            if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
        }
        & "$ModelSimBin\vlog.exe" -work work +incdir+../../rtl/common +incdir+../../rtl/math ../../rtl/math/fp_divsqrt.v ../../rtl/math/fp_operator.v ../../rtl/math/jacobi_eigen.v
        if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
        & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../rtl/common ../tb_jacobi_eigen.sv
        if ($LASTEXITCODE -ne 0) { throw 'Testbench compilation failed' }
        & "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_jacobi_eigen -do ../run_jacobi_eigen.do
        if ($LASTEXITCODE -ne 0) { throw 'Jacobi simulation failed; see build_eigen/eigen_results.txt' }
        if (!(Select-String -LiteralPath simulation.log -SimpleMatch JACOBI_EIGEN_PASS -Quiet)) {
            throw 'Missing simulation pass marker'
        }
        Get-Content -LiteralPath eigen_results.txt
    } finally { Pop-Location }
} finally { Pop-Location }
