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
    if (!(Test-Path -LiteralPath build_init_controller)) { New-Item -ItemType Directory -Path build_init_controller | Out-Null }
    Push-Location (Join-Path $BuildRoot 'build_init_controller')
    try {
        if (!(Test-Path -LiteralPath work)) {
            & "$ModelSimBin\vlib.exe" work
            if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
        }
        & "$ModelSimBin\vlog.exe" -work work +incdir+../../../rtl/include +incdir+../../../rtl/compute/float ../../../rtl/compute/float/fp_divsqrt.v ../../../rtl/compute/float/fp_operator.v ../../../rtl/compute/service/fp_calibration_pool.v ../../../rtl/compute/linalg/jacobi_eigen.v ../../../rtl/compute/service/eigen_endpoint.v ../../../rtl/compute/geometry/rotation.v ../../../rtl/memory/parameters/corner_store.v ../../../rtl/control/calibration/init/homography.v ../../../rtl/control/calibration/init/zhang.v ../../../rtl/control/calibration/init/pose_init.v ../../../rtl/control/calibration/init/init_controller.v
        if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
        & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../../rtl/include ../../../tb/calibration/tb_init_controller.sv
        if ($LASTEXITCODE -ne 0) { throw 'Testbench compilation failed' }
        Invoke-Simulator "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_init_controller -do ../../../scripts/calibration/run_init_controller.do
        if ($LASTEXITCODE -ne 0) { throw 'init_controller simulation failed; see build_init_controller' }
        if (!(Select-String -LiteralPath simulation.log -SimpleMatch INIT_CONTROLLER_PASS -Quiet)) { throw 'Missing simulation pass marker' }
        Get-Content -Path *_results.txt
    } finally { Pop-Location }
} finally { Pop-Location }
