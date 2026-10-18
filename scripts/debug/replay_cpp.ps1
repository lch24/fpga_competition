param([string]$VsRoot=$env:VS_ROOT)
$ErrorActionPreference='Stop'
if(!$VsRoot){throw 'Set VS_ROOT or pass -VsRoot'}
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$out=Join-Path $root 'build/debug/cpp_replay'
New-Item -ItemType Directory -Force $out | Out-Null
$report=Get-Content (Join-Path $root 'build/debug/calibration_replay.json') -Raw | ConvertFrom-Json
$source=$report.source
if(![IO.Path]::IsPathRooted($source)){$source=Join-Path $root $source}
$capture=Get-Content $source -Raw | ConvertFrom-Json
$lines=@("$($report.width) $($report.height) $($report.rows) $($report.cols) $($report.views)")
$culture=[Globalization.CultureInfo]::InvariantCulture
foreach($p in $capture.corners){$lines+=([double]$p.xy[0]).ToString('R',$culture)+' '+([double]$p.xy[1]).ToString('R',$culture)}
[IO.File]::WriteAllLines((Join-Path $out 'corners.txt'),$lines)
$base='algorithom/closer2fpga/closer2fpga'
$sources=@('data/generators/calibration/replay_uart.cpp',"$base/algo/calibrate.cpp",
 "$base/algo/calibration/report.cpp","$base/algo/calibration/residuals.cpp",
 "$base/algo/calibration/initialization.cpp","$base/algo/calibration/pose_init.cpp",
 "$base/algo/calibration/lm.cpp","$base/common/math3.cpp","$base/common/matrix.cpp")
$quoted=($sources | ForEach-Object {'"'+(Join-Path $root $_)+'"'}) -join ' '
$batch=@"
@echo off
call "$VsRoot\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /O2 /fp:strict /utf-8 $quoted /Fe:replay.exe
if errorlevel 1 exit /b 1
replay.exe corners.txt > result.json
"@
[IO.File]::WriteAllText((Join-Path $out 'run.cmd'),$batch)
Push-Location $out
try { & cmd /c run.cmd; if($LASTEXITCODE){throw 'C++ replay failed'}; Get-Content result.json }
finally {Pop-Location}
