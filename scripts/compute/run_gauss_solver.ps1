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
    if (!(Test-Path -LiteralPath build_gauss)) { New-Item -ItemType Directory -Path build_gauss | Out-Null }
    Push-Location (Join-Path $BuildRoot 'build_gauss')
    try {
        if (!(Test-Path -LiteralPath work)) {
            & "$ModelSimBin\vlib.exe" work
            if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
        }
        & "$ModelSimBin\vlog.exe" -work work +incdir+../../../rtl/include +incdir+../../../rtl/compute/float ../../../rtl/compute/float/fp_divsqrt.v ../../../rtl/compute/float/fp_operator.v ../../../rtl/compute/service/fp_calibration_pool.v ../../../rtl/compute/linalg/gauss_solver.v
        if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
        & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../../rtl/include ../../../tb/compute/tb_gauss_solver.sv
        if ($LASTEXITCODE -ne 0) { throw 'Testbench compilation failed' }
        Invoke-Simulator "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_gauss_solver -do ../../../scripts/compute/run_gauss_solver.do
        if ($LASTEXITCODE -ne 0) { throw 'Gauss simulation failed; see build_gauss/gauss_results.txt' }
        if (!(Select-String -LiteralPath simulation.log -SimpleMatch GAUSS_SOLVER_PASS -Quiet)) {
            throw 'Missing simulation pass marker'
        }
        Get-Content -LiteralPath gauss_results.txt
    } finally { Pop-Location }
} finally { Pop-Location }
