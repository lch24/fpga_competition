# jpg_to_bgr.ps1 - decode test JPGs into raw BGR byte files (no external deps)
# ---------------------------------------------------------------------------
# Uses .NET System.Drawing (GDI+), which is available in Windows PowerShell by
# default. Output files are consumed by export_vectors.exe:
#   testN.bgr : raw bytes, BGR order, row-major, no padding (W*H*3 bytes)
#   testN.dim : ASCII text "W H"
# ---------------------------------------------------------------------------
param(
    [string]$Root = "..\..",
    [string]$OutDir = "..\build\raw"
)

$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Drawing

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

for ($i = 0; $i -lt 3; $i++) {
    $src = Join-Path $Root "test$i.jpg"
    if (-not (Test-Path $src)) {
        Write-Error "missing input: $src"
        exit 2
    }

    $bmp = [System.Drawing.Bitmap]::FromFile($src)
    $w = $bmp.Width
    $h = $bmp.Height
    $rect = New-Object System.Drawing.Rectangle(0, 0, $w, $h)
    $fmt = [System.Drawing.Imaging.PixelFormat]::Format24bppRgb
    $data = $bmp.LockBits($rect, [System.Drawing.Imaging.ImageLockMode]::ReadOnly, $fmt)
    $strideBytes = $data.Stride
    $buf = New-Object byte[] ($strideBytes * $h)
    [System.Runtime.InteropServices.Marshal]::Copy($data.Scan0, $buf, 0, $buf.Length)
    $bmp.UnlockBits($data)
    $bmp.Dispose()

    # strip per-row stride padding
    $rowBytes = $w * 3
    $out = New-Object byte[] ($rowBytes * $h)
    if ($strideBytes -eq $rowBytes) {
        $out = $buf
    } else {
        for ($y = 0; $y -lt $h; $y++) {
            [Array]::Copy($buf, $y * $strideBytes, $out, $y * $rowBytes, $rowBytes)
        }
    }

    $bgrPath = Join-Path $OutDir "test$i.bgr"
    $dimPath = Join-Path $OutDir "test$i.dim"
    [System.IO.File]::WriteAllBytes($bgrPath, $out)
    "$w $h" | Out-File -FilePath $dimPath -Encoding ascii

    Write-Output "test$i.jpg -> test$i.bgr ($w x $h)"
}
exit 0
