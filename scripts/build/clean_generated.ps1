# Run from any directory. Use -WhatIf to preview; never removes source/test data.
[CmdletBinding(SupportsShouldProcess=$true)]
param()
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..')).TrimEnd('\')
$prefix=$root+'\'
$relative=@('build','sim/modelsim_smoke','logbackup',
 'OV5640_DualView_100H/logbackup','undistort/sim/build')
$paths=@($relative | ForEach-Object {[IO.Path]::GetFullPath((Join-Path $root $_))} |
 Where-Object {Test-Path -LiteralPath $_})
# Validate every final absolute path before any recursive deletion.
foreach($p in $paths){
 if(!$p.StartsWith($prefix,[StringComparison]::OrdinalIgnoreCase)){throw "Outside workspace: $p"}
 $entries=@(Get-Item -LiteralPath $p)+@(Get-ChildItem -LiteralPath $p -Force -Recurse)
 if(@($entries | Where-Object {$_.Attributes -band [IO.FileAttributes]::ReparsePoint}).Count){throw "Refusing linked directory: $p"}
}
# Protect tracked data even if someone places it in a generated directory later.
$tracked=@(& git -C $root ls-files)
if($LASTEXITCODE -ne 0){throw 'Cannot verify tracked files'}
foreach($p in $paths){
 $rel=$p.Substring($prefix.Length).Replace('\','/')+'/'
 if(@($tracked | Where-Object {$_.StartsWith($rel,[StringComparison]::OrdinalIgnoreCase)}).Count){throw "Tracked files in cleanup target: $p"}
}
$removedFiles=0;$removedBytes=0L;$removedDirs=0
foreach($p in $paths){
 $files=@(Get-ChildItem -LiteralPath $p -File -Recurse -Force)
 $bytes=($files | Measure-Object -Property Length -Sum).Sum
 if($PSCmdlet.ShouldProcess($p,'Remove generated files')){
  Remove-Item -LiteralPath $p -Recurse -Force
  $removedFiles+=$files.Count;$removedBytes+=$bytes;$removedDirs++
 }
}
[pscustomobject]@{Directories=$removedDirs;Files=$removedFiles;MiB=[math]::Round($removedBytes/1MB,2)}
