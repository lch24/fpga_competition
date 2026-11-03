param(
    [string]$VsRoot = $env:VS_ROOT,
    [string]$VcpkgRoot = $env:VCPKG_ROOT,
    [string]$InputDirectory = 'algorithom',
    [string]$Output = '',
    [int]$Columns = 8,
    [int]$Rows = 5
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
if (!$Output) { $Output = 'build/light/run_' + (Get-Date -Format 'yyyyMMdd_HHmmss') }
if (![IO.Path]::IsPathRooted($Output)) { $Output = Join-Path $repo $Output }
if (![IO.Path]::IsPathRooted($InputDirectory)) { $InputDirectory = Join-Path $repo $InputDirectory }
if (Test-Path -LiteralPath $Output) { throw 'Output exists; select a new directory.' }
$names = 'an.png','an2.png','an3.png','ming.png','ming2.png','ming3.png'
$hashes = @($names | ForEach-Object {
    $file = Get-Item -LiteralPath (Join-Path $InputDirectory $_)
    [ordered]@{image=$_.ToString(); bytes=$file.Length; sha256=(Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant()}
})
$build = Join-Path $repo 'build/light/compiler'
New-Item -ItemType Directory -Force $build | Out-Null
$core = Join-Path $repo 'algorithom/closer2fpga/closer2fpga'
$sources = @(
    (Join-Path $PSScriptRoot 'light/main.cpp'),
    (Join-Path $PSScriptRoot 'light/experiment.cpp'),
    (Join-Path $core 'algo/chessboard.cpp'),
    (Join-Path $core 'algo/chessboard/candidates.cpp'),
    (Join-Path $core 'algo/chessboard/ordering.cpp'),
    (Join-Path $core 'algo/chessboard/validation.cpp'),
    (Join-Path $core 'algo/subpixel.cpp')
)
$sourceArgs = ($sources | ForEach-Object { '"' + $_ + '"' }) -join ' '
$batch = @"
@echo off
call "$VsRoot\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /O2 /MD /utf-8 /W4 /I"$sdk\include\opencv4" $sourceArgs /Fe:light.exe /link /LIBPATH:"$sdk\lib" opencv_core4.lib opencv_imgproc4.lib opencv_imgcodecs4.lib opencv_calib3d4.lib
exit /b %errorlevel%
"@
[IO.File]::WriteAllText((Join-Path $build 'compile.cmd'), $batch)
Push-Location $build
try {
    & cmd /c compile.cmd
    if ($LASTEXITCODE) { throw 'C++ compilation failed.' }
} finally { Pop-Location }
$oldPath = $env:PATH
try {
    $env:PATH = (Join-Path $sdk 'bin') + ';' + $env:PATH
    & (Join-Path $build 'light.exe') --self-test
    if ($LASTEXITCODE) { throw 'Calibration/correspondence self-test failed.' }
    & (Join-Path $build 'light.exe') $InputDirectory $Output $Columns $Rows
    if ($LASTEXITCODE) { throw 'Brightness experiment failed; inspect partial output.' }
    foreach ($item in $hashes) {
        if ((Get-FileHash -LiteralPath (Join-Path $InputDirectory $item.image) -Algorithm SHA256).Hash.ToLowerInvariant() -ne $item.sha256) {
            throw 'Input image changed during execution; discard this run.'
        }
    }
    $provenance = [ordered]@{
        completed_local = (Get-Date -Format 'yyyy-MM-ddTHH:mm:sszzz')
        columns = $Columns
        rows = $Rows
        inputs = $hashes
        sources = @($sources | ForEach-Object {
            [ordered]@{path=$_.Substring($repo.Length + 1).Replace('\','/'); sha256=(Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash.ToLowerInvariant()}
        })
    }
    [IO.File]::WriteAllText((Join-Path $Output 'input_sha256.json'), ($provenance | ConvertTo-Json -Depth 5), (New-Object Text.UTF8Encoding $false))
} finally { $env:PATH = $oldPath }
