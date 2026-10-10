param([string]$ModelSimBin=$env:MODELSIM_BIN)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot '../common/tools.ps1')
$ModelSimBin=Resolve-ToolDirectory $ModelSimBin 'vsim.exe' 'MODELSIM_BIN'
$ScriptDir=$PSScriptRoot
$RepoRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$BuildRoot=Join-Path $RepoRoot 'build/compute'
New-Item -ItemType Directory -Force $BuildRoot | Out-Null

$build=Join-Path $BuildRoot 'build_fp_profiles'
New-Item -ItemType Directory -Force $build | Out-Null
Push-Location $build
try {
    if (!(Test-Path work)) { & "$ModelSimBin\vlib.exe" work; if($LASTEXITCODE){throw 'vlib failed'} }
    & "$ModelSimBin\vlog.exe" -sv +incdir+../../../rtl/include +incdir+../../../rtl/compute/float ../../../rtl/compute/float/fp_divsqrt.v ../../../rtl/compute/float/calib_alu.v ../../../rtl/compute/float/fp_math_program.v ../../../rtl/compute/float/fp_operator.v ../../../rtl/compute/service/fp_calibration_pool.v ../../../tb/compute/tb_fp_operator.sv ../../../tb/compute/tb_fp_profiles.sv
    if($LASTEXITCODE){throw 'compile failed'}
    Invoke-Simulator "$ModelSimBin\vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_fp_profiles -do ../../../scripts/compute/run_fp_profiles.do
    if($LASTEXITCODE){throw 'profile simulation failed'}
    if(!(Select-String -LiteralPath simulation.log -Pattern FP_PROFILES_PASS -SimpleMatch -Quiet)){throw 'Missing pass marker'}
    Select-String -Path '*_results.txt' -Pattern '^RESULT'
} finally { Pop-Location }
