param(
    [string]$VsRoot = $env:VS_ROOT,
    [string]$VcpkgRoot = $env:VCPKG_ROOT,
    [string]$Output = 'build/opencv_comparison/detection',
    [string]$Manifest = ''
)
$ErrorActionPreference = 'Stop'
$repo = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
if (!$VsRoot) {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
    if (Test-Path $vswhere) {
        $VsRoot = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    }
}
if (!$VsRoot) { throw 'Pass -VsRoot or set VS_ROOT to your Visual Studio installation.' }
if (!$VcpkgRoot) { $VcpkgRoot = Join-Path $VsRoot 'vcpkg' }
$sdk = Join-Path $VcpkgRoot 'installed/x64-windows'
if (!(Test-Path (Join-Path $sdk 'include/opencv4/opencv2/calib3d.hpp'))) { throw 'OpenCV x64-windows is missing; set VCPKG_ROOT.' }
$build = Join-Path $repo 'build/opencv_comparison/compiler'
New-Item -ItemType Directory -Force $build | Out-Null
$source = Join-Path $PSScriptRoot 'modification/main.cpp'
$compare = Join-Path $PSScriptRoot 'modification/compare.cpp'
$batch = @"
@echo off
call "$VsRoot\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /O2 /utf-8 /W4 /I"$sdk\include\opencv4" "$source" "$compare" /Fe:modification.exe /link /LIBPATH:"$sdk\lib" opencv_core4.lib opencv_imgproc4.lib opencv_imgcodecs4.lib opencv_calib3d4.lib
exit /b %errorlevel%
"@
[IO.File]::WriteAllText((Join-Path $build 'compile.cmd'), $batch)
Push-Location $build
try {
    & cmd /c compile.cmd
    if ($LASTEXITCODE) { throw 'C++ compilation failed' }
} finally { Pop-Location }
$oldPath = $env:PATH
try {
    $env:PATH = (Join-Path $sdk 'bin') + ';' + $env:PATH
    & (Join-Path $build 'modification.exe') --self-test
    if ($LASTEXITCODE) { throw 'Synthetic calibration consistency check failed' }
    if (![IO.Path]::IsPathRooted($Output)) { $Output = Join-Path $repo $Output }
    if ($Manifest) {
        if (![IO.Path]::IsPathRooted($Manifest)) { $Manifest = Join-Path $repo $Manifest }
        & (Join-Path $build 'modification.exe') --compare $Manifest $Output
    } else {
        & (Join-Path $build 'modification.exe') (Join-Path $repo 'algorithom') $Output
    }
    if ($LASTEXITCODE) { throw 'OpenCV reference run failed; inspect its output.' }
} finally { $env:PATH = $oldPath }
