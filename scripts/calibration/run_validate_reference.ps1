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
    $build=Join-Path $BuildRoot 'build_validate_reference'
    if (!(Test-Path -LiteralPath $build)) { New-Item -ItemType Directory -Path $build | Out-Null }
    $vcvars=Join-Path $VsRoot 'VC\Auxiliary\Build\vcvars64.bat'
    if (!(Test-Path -LiteralPath $vcvars)) { throw "Missing MSVC environment: $vcvars" }
    $batch=@"
@echo off
call "$vcvars" >nul
if errorlevel 1 exit /b 1
cd /d "$build"
cl /nologo /utf-8 /EHsc /std:c++17 /fp:strict /Fe:generate_validate_reference.exe ..\..\..\data\generators\calibration\generate_validate_reference.cpp ..\..\..\algorithom\closer2fpga\closer2fpga\algo\calibration\report.cpp ..\..\..\algorithom\closer2fpga\closer2fpga\algo\calibration\residuals.cpp ..\..\..\algorithom\closer2fpga\closer2fpga\common\matrix.cpp ..\..\..\algorithom\closer2fpga\closer2fpga\common\math3.cpp
exit /b %errorlevel%
"@
    $batchPath=Join-Path $build 'compile.cmd'
    Set-Content -LiteralPath $batchPath -Value $batch -Encoding Default
    & $env:ComSpec /d /c $batchPath
    if ($LASTEXITCODE -ne 0) { throw 'Reference compilation failed' }
    & (Join-Path $build 'generate_validate_reference.exe')
    if ($LASTEXITCODE -ne 0) { throw 'C++ reference generation failed' }
} finally { Pop-Location }
