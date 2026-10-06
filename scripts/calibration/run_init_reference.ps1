param([string]$VsRoot=$env:VS_ROOT)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot '../common/tools.ps1')
if(!$VsRoot){throw 'Set VS_ROOT or pass -VsRoot (Visual Studio installation directory)'}
$ScriptDir=$PSScriptRoot
$RepoRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$BuildRoot=Join-Path $RepoRoot 'build/calibration'
New-Item -ItemType Directory -Force $BuildRoot | Out-Null

Push-Location (Join-Path $RepoRoot 'data/calibration')
try {
    $build=Join-Path $BuildRoot 'build_init_reference'
    if (!(Test-Path -LiteralPath $build)) { New-Item -ItemType Directory -Path $build | Out-Null }
    $vcvars=Join-Path $VsRoot 'VC\Auxiliary\Build\vcvars64.bat'
    if (!(Test-Path -LiteralPath $vcvars)) { throw "Missing MSVC environment: $vcvars" }
    # Batch file contains only fixed compiler arguments and quoted literal paths.
    $batch=@"
@echo off
call "$vcvars" >nul
if errorlevel 1 exit /b 1
cd /d "$build"
cl /nologo /EHsc /std:c++17 /fp:strict /Fe:verify_init_reference.exe ..\..\..\data\generators\calibration\verify_init_reference.cpp ..\..\..\algorithom\closer2fpga\closer2fpga\algo\calibration\initialization.cpp ..\..\..\algorithom\closer2fpga\closer2fpga\algo\calibration\pose_init.cpp ..\..\..\algorithom\closer2fpga\closer2fpga\common\math3.cpp ..\..\..\algorithom\closer2fpga\closer2fpga\common\symmetric_eigen.cpp
exit /b %errorlevel%
"@
    $batchPath=Join-Path $build 'compile.cmd'
    Set-Content -LiteralPath $batchPath -Value $batch -Encoding Default
    & $env:ComSpec /d /c $batchPath
    if ($LASTEXITCODE -ne 0) { throw 'Reference compilation failed' }
    & (Join-Path $build 'verify_init_reference.exe')
    if ($LASTEXITCODE -ne 0) { throw 'C++ reference validation failed' }
} finally { Pop-Location }
