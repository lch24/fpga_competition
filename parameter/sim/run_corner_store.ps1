param(
    [string]$ModelSimBin = 'E:\pangu\Modelsim10.1c\win64'
)
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    if (!(Test-Path -LiteralPath 'build')) {
        New-Item -ItemType Directory -Path build | Out-Null
    }
    Push-Location (Join-Path $PSScriptRoot 'build')
    try {
        if (!(Test-Path -LiteralPath 'work')) {
            & "$ModelSimBin\vlib.exe" work
            if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
        }
        & "$ModelSimBin\vlog.exe" -work work +incdir+../../rtl/common ../../rtl/control/corner_store.v
        if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
        & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../rtl/common ../tb_corner_store.sv
        if ($LASTEXITCODE -ne 0) { throw 'Testbench compilation failed' }
        & "$ModelSimBin\vsim.exe" -c -voptargs=+acc -wlf corner_store.wlf -l simulation.log work.tb_corner_store -do ../run_corner_store.do
        if ($LASTEXITCODE -ne 0) { throw 'Simulation failed; see build/results.txt and simulation.log' }
        if (!(Select-String -LiteralPath simulation.log -SimpleMatch 'CORNER_STORE_PASS' -Quiet)) {
            throw 'Simulation did not report CORNER_STORE_PASS'
        }
        Get-Content -LiteralPath results.txt
    } finally { Pop-Location }
} finally { Pop-Location }
