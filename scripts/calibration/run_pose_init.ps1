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
    if (!(Test-Path -LiteralPath build_pose_init)) { New-Item -ItemType Directory -Path build_pose_init | Out-Null }
    Push-Location (Join-Path $BuildRoot 'build_pose_init')
    try {
        if (!(Test-Path -LiteralPath work)) {
            & "$ModelSimBin\vlib.exe" work
            if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
        }
        & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../../rtl/include +incdir+../../../rtl/compute/float ../../../rtl/compute/float/fp_divsqrt.v ../../../rtl/compute/float/fp_math_program.v ../../../rtl/compute/float/fp_operator.v ../../../rtl/compute/service/fp_calibration_pool.v ../../../rtl/compute/geometry/rotation.v ../../../rtl/memory/parameters/corner_store.v ../../../rtl/control/calibration/init/pose_init.v ../../../rtl/control/calibration/init/init_controller.v ../../../rtl/compute/float/calib_alu.v ../../../rtl/control/calibration/engine/calib_sequencer.v ../../../rtl/control/calibration/engine/calib_kernel_ctrl.v ../../../rtl/memory/local/calib_workspace.v ../../../rtl/compute/service/calib_datapath.v
        if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
        & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../../rtl/include ../../../tb/calibration/tb_pose_init.sv
        if ($LASTEXITCODE -ne 0) { throw 'Testbench compilation failed' }
        Invoke-Simulator "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_pose_init -do ../../../scripts/calibration/run_pose_init.do
        if ($LASTEXITCODE -ne 0) { throw 'pose_init simulation failed; see build_pose_init' }
        if (!(Select-String -LiteralPath simulation.log -SimpleMatch POSE_INIT_PASS -Quiet)) { throw 'Missing simulation pass marker' }
        Get-Content -Path *_results.txt
    } finally { Pop-Location }
} finally { Pop-Location }
