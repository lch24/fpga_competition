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
    if (!(Test-Path -LiteralPath build_project_point)) { New-Item -ItemType Directory -Path build_project_point | Out-Null }
    Push-Location (Join-Path $BuildRoot 'build_project_point')
    try {
        if (!(Test-Path -LiteralPath work)) {
            & "$ModelSimBin\vlib.exe" work
            if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
        }
        & "$ModelSimBin\vlog.exe" -work work +incdir+../../../rtl/include +incdir+../../../rtl/compute/float ../../../rtl/compute/float/fp_divsqrt.v ../../../rtl/compute/float/calib_alu.v ../../../rtl/compute/float/fp_math_program.v ../../../rtl/compute/float/fp_operator.v ../../../rtl/compute/service/fp_calibration_pool.v ../../../rtl/memory/local/work_ram.v ../../../rtl/compute/geometry/geometry_engine.v ../../../rtl/compute/geometry/brown_distort.v ../../../rtl/compute/geometry/rotation.v ../../../rtl/compute/geometry/project_point.v ../../../rtl/compute/geometry/residual_engine.v ../../../rtl/compute/service/residual_endpoint.v
        if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
        & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../../rtl/include ../../../tb/compute/tb_project_point.sv
        if ($LASTEXITCODE -ne 0) { throw 'Testbench compilation failed' }
        Invoke-Simulator "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_project_point -do ../../../scripts/compute/run_project_point.do
        if ($LASTEXITCODE -ne 0) { throw 'project_point simulation failed; see build_project_point' }
        if (!(Select-String -LiteralPath simulation.log -SimpleMatch PROJECT_POINT_PASS -Quiet)) { throw 'Missing simulation pass marker' }
        Get-Content -Path *_results.txt
    } finally { Pop-Location }
} finally { Pop-Location }
