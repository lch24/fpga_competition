param([string]$ModelSimBin=$env:MODELSIM_BIN)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot '../common/tools.ps1')
$ModelSimBin=Resolve-ToolDirectory $ModelSimBin 'vsim.exe' 'MODELSIM_BIN'
$RepoRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$BuildRoot=Join-Path $RepoRoot 'build/compute/calib_alu'
New-Item -ItemType Directory -Force $BuildRoot | Out-Null
Push-Location $BuildRoot
try {
 if (!(Test-Path work)) { & "$ModelSimBin/vlib.exe" work; if($LASTEXITCODE){throw 'vlib failed'} }
 & "$ModelSimBin/vlog.exe" -sv +incdir+../../../rtl/include ../../../rtl/compute/float/fp_divsqrt.v ../../../rtl/compute/float/calib_alu.v ../../../tb/compute/tb_fp_operator.sv ../../../tb/compute/tb_calib_alu.sv
 if($LASTEXITCODE){throw 'compile failed'}
 Invoke-Simulator "$ModelSimBin/vsim.exe" -c -voptargs=+acc -l simulation.log work.tb_calib_alu -do ../../../scripts/compute/run_calib_alu.do
 if($LASTEXITCODE){throw 'ALU simulation failed'}
 if(!(Select-String -LiteralPath simulation.log -Pattern CALIB_ALU_PASS -SimpleMatch -Quiet)){throw 'Missing pass marker'}
} finally { Pop-Location }
