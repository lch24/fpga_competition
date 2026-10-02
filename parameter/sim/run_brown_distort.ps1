param([string]$ModelSimBin='E:\pangu\Modelsim10.1c\win64')
$ErrorActionPreference='Stop'
Push-Location $PSScriptRoot
try {
    if (!(Test-Path -LiteralPath build_brown_distort)) { New-Item -ItemType Directory -Path build_brown_distort | Out-Null }
    Push-Location (Join-Path $PSScriptRoot 'build_brown_distort')
    try {
        if (!(Test-Path -LiteralPath work)) {
            & "$ModelSimBin\vlib.exe" work
            if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
        }
        & "$ModelSimBin\vlog.exe" -work work +incdir+../../rtl/common +incdir+../../rtl/math ../../rtl/math/fp_divsqrt.v ../../rtl/math/fp_operator.v ../../rtl/model/brown_distort.v ../../rtl/model/rotation.v ../../rtl/model/project_point.v ../../rtl/model/residual_engine.v
        if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
        & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../rtl/common ../tb_brown_distort.sv
        if ($LASTEXITCODE -ne 0) { throw 'Testbench compilation failed' }
        & "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_brown_distort -do ../run_brown_distort.do
        if ($LASTEXITCODE -ne 0) { throw 'brown_distort simulation failed; see build_brown_distort' }
        if (!(Select-String -LiteralPath simulation.log -SimpleMatch BROWN_DISTORT_PASS -Quiet)) { throw 'Missing simulation pass marker' }
        Get-Content -Path *_results.txt
    } finally { Pop-Location }
} finally { Pop-Location }
