param([string]$ModelSimBin='E:\pangu\Modelsim10.1c\win64')
$ErrorActionPreference='Stop'
foreach ($module in @('homography','zhang','pose_init','init_controller')) {
    & (Join-Path $PSScriptRoot "run_$module.ps1") -ModelSimBin $ModelSimBin
}
