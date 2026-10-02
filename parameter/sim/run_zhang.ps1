param([string]$ModelSimBin='E:\pangu\Modelsim10.1c\win64')
$ErrorActionPreference='Stop'
Push-Location $PSScriptRoot
try {
    if (!(Test-Path -LiteralPath build_zhang)) { New-Item -ItemType Directory -Path build_zhang | Out-Null }
    Push-Location (Join-Path $PSScriptRoot 'build_zhang')
    try {
        if (!(Test-Path -LiteralPath work)) {
            & "$ModelSimBin\vlib.exe" work
            if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
        }
        & "$ModelSimBin\vlog.exe" -work work +incdir+../../rtl/common +incdir+../../rtl/math ../../rtl/math/fp_divsqrt.v ../../rtl/math/fp_operator.v ../../rtl/math/jacobi_eigen.v ../../rtl/model/rotation.v ../../rtl/control/corner_store.v ../../rtl/calibration/init/homography.v ../../rtl/calibration/init/zhang.v ../../rtl/calibration/init/pose_init.v ../../rtl/calibration/init/init_controller.v
        if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
        & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../rtl/common ../tb_zhang.sv
        if ($LASTEXITCODE -ne 0) { throw 'Testbench compilation failed' }
        & "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_zhang -do ../run_zhang.do
        if ($LASTEXITCODE -ne 0) { throw 'zhang simulation failed; see build_zhang' }
        if (!(Select-String -LiteralPath simulation.log -SimpleMatch ZHANG_PASS -Quiet)) { throw 'Missing simulation pass marker' }
        Get-Content -Path *_results.txt
    } finally { Pop-Location }
} finally { Pop-Location }
