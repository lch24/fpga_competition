param(
    [string]$ModelSimBin = $env:MODELSIM_BIN
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot '../common/tools.ps1')
$ModelSimBin=Resolve-ToolDirectory $ModelSimBin 'vsim.exe' 'MODELSIM_BIN'
$ScriptDir=$PSScriptRoot
$RepoRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$BuildRoot=Join-Path $RepoRoot 'build/memory'
New-Item -ItemType Directory -Force $BuildRoot | Out-Null

Push-Location $BuildRoot
try {
    if (!(Test-Path -LiteralPath 'build')) {
        New-Item -ItemType Directory -Path build | Out-Null
    }
    Push-Location (Join-Path $BuildRoot 'build')
    try {
        if (!(Test-Path -LiteralPath 'work')) {
            & "$ModelSimBin\vlib.exe" work
            if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
        }
        & "$ModelSimBin\vlog.exe" -work work +incdir+../../../rtl/include ../../../rtl/memory/parameters/corner_store.v
        if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
        & "$ModelSimBin\vlog.exe" -sv -work work +incdir+../../../rtl/include ../../../tb/memory/tb_corner_store.sv
        if ($LASTEXITCODE -ne 0) { throw 'Testbench compilation failed' }
        Invoke-Simulator "$ModelSimBin\vsim.exe" -c -voptargs=+acc -wlf corner_store.wlf -l simulation.log work.tb_corner_store -do ../../../scripts/memory/run_corner_store.do
        if ($LASTEXITCODE -ne 0) { throw 'Simulation failed; see build/results.txt and simulation.log' }
        if (!(Select-String -LiteralPath simulation.log -SimpleMatch 'CORNER_STORE_PASS' -Quiet)) {
            throw 'Simulation did not report CORNER_STORE_PASS'
        }
        Get-Content -LiteralPath results.txt
    } finally { Pop-Location }
} finally { Pop-Location }
