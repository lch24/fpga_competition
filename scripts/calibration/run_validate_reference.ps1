param([string]$VsRoot=$env:VS_ROOT)
$ErrorActionPreference='Stop'
if(!$VsRoot){throw 'Set VS_ROOT or pass -VsRoot with the Visual Studio installation directory'}
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$build=Join-Path $root 'build/validate_reference'
New-Item -ItemType Directory -Force $build | Out-Null
$sources=@('data/generators/calibration/generate_validate_reference.cpp',
 'algorithom/closer2fpga/closer2fpga/algo/calibration/report.cpp',
 'algorithom/closer2fpga/closer2fpga/algo/calibration/residuals.cpp',
 'algorithom/closer2fpga/closer2fpga/common/math3.cpp')
$quoted=($sources | ForEach-Object {'"'+(Join-Path $root $_)+'"'}) -join ' '
$batch=@"
@echo off
call "$VsRoot\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /O2 /fp:strict /utf-8 $quoted /Fe:reference.exe
if errorlevel 1 exit /b 1
reference.exe
"@
[IO.File]::WriteAllText((Join-Path $build 'run.cmd'),$batch)
Push-Location $build
try { & cmd /c run.cmd; if($LASTEXITCODE){throw 'Reference generation failed'} } finally {Pop-Location}
Copy-Item (Join-Path $build 'validate_result_vectors.txt') (Join-Path $root 'data/calibration')
Copy-Item (Join-Path $build 'validate_result_cases.txt') (Join-Path $root 'data/calibration')
