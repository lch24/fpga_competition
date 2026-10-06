param([string]$ModelSimBin=$env:MODELSIM_BIN)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot '../common/tools.ps1')
$ModelSimBin=Resolve-ToolDirectory $ModelSimBin 'vsim.exe' 'MODELSIM_BIN'
$ScriptDir=$PSScriptRoot
$RepoRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$BuildRoot=Join-Path $RepoRoot 'build/compute'
New-Item -ItemType Directory -Force $BuildRoot | Out-Null

Push-Location $BuildRoot
try {
    if (!(Test-Path -LiteralPath build_eigen)) { New-Item -ItemType Directory -Path build_eigen | Out-Null }
    Push-Location (Join-Path $BuildRoot 'build_eigen')
    try {
        if (!(Test-Path -LiteralPath work)) {
            & "$ModelSimBin\vlib.exe" work
            if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
        }
        & "$ModelSimBin\vlog.exe" -work work +incdir+../../../rtl/include +incdir+../../../rtl/compute/float ../../../rtl/compute/float/fp_divsqrt.v ../../../rtl/compute/float/fp_operator.v ../../../rtl/compute/service/fp_calibration_pool.v ../../../rtl/compute/linalg/jacobi_eigen.v ../../../rtl/compute/service/eigen_endpoint.v
        if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
        & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../../rtl/include ../../../tb/compute/tb_jacobi_eigen.sv
        if ($LASTEXITCODE -ne 0) { throw 'Testbench compilation failed' }
        Invoke-Simulator "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_jacobi_eigen -do ../../../scripts/compute/run_jacobi_eigen.do
        if ($LASTEXITCODE -ne 0) { throw 'Jacobi simulation failed; see build_eigen/eigen_results.txt' }
        if (!(Select-String -LiteralPath simulation.log -SimpleMatch JACOBI_EIGEN_PASS -Quiet)) {
            throw 'Missing simulation pass marker'
        }
        Get-Content -LiteralPath eigen_results.txt
    } finally { Pop-Location }
} finally { Pop-Location }
