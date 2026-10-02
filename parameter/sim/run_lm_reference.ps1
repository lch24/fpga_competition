param([string]$VsRoot='E:\vs')
$ErrorActionPreference='Stop'
Push-Location $PSScriptRoot
try {
    $build=Join-Path $PSScriptRoot 'build_lm_reference'
    if (!(Test-Path -LiteralPath $build)) { New-Item -ItemType Directory -Path $build | Out-Null }
    $vcvars=Join-Path $VsRoot 'VC\Auxiliary\Build\vcvars64.bat'
    if (!(Test-Path -LiteralPath $vcvars)) { throw "Missing MSVC environment: $vcvars" }
    $batch=@"
@echo off
call "$vcvars" >nul
if errorlevel 1 exit /b 1
cd /d "$build"
cl /nologo /EHsc /std:c++17 /fp:strict /Fe:generate_lm_reference.exe ..\generate_lm_reference.cpp ..\..\..\algorithom\closer2fpga\closer2fpga\algo\calibration\lm.cpp ..\..\..\algorithom\closer2fpga\closer2fpga\algo\calibration\residuals.cpp ..\..\..\algorithom\closer2fpga\closer2fpga\common\matrix.cpp ..\..\..\algorithom\closer2fpga\closer2fpga\common\math3.cpp
exit /b %errorlevel%
"@
    $batchPath=Join-Path $build 'compile.cmd'
    Set-Content -LiteralPath $batchPath -Value $batch -Encoding Default
    & $env:ComSpec /d /c $batchPath
    if ($LASTEXITCODE -ne 0) { throw 'Reference compilation failed' }
    & (Join-Path $build 'generate_lm_reference.exe')
    if ($LASTEXITCODE -ne 0) { throw 'C++ reference generation failed' }
} finally { Pop-Location }
