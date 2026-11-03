# Clean generated outputs from any working directory; -WhatIf previews the scope.
[CmdletBinding(SupportsShouldProcess=$true)]
param()
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..')).TrimEnd('\')
$prefix=$root+'\'

function Assert-LocalPath([string]$path) {
 $full=[IO.Path]::GetFullPath($path)
 if(!$full.StartsWith($prefix,[StringComparison]::OrdinalIgnoreCase)){throw "Outside workspace: $path"}
 $part=Get-Item -LiteralPath $full -Force
 while($part -and $part.FullName -ne $root){
  if($part.Attributes -band [IO.FileAttributes]::ReparsePoint){throw "Refusing linked path: $($part.FullName)"}
  $part=if($part.PSIsContainer){$part.Parent}else{$part.Directory}
 }
}
# Preserve tracked files, bitstreams, and generated files referenced by saved projects.
$protected=New-Object 'System.Collections.Generic.HashSet[string]' ([StringComparer]::OrdinalIgnoreCase)
$tracked=@(& git -C $root -c core.quotepath=false ls-files)
if($LASTEXITCODE -ne 0){throw 'Cannot verify tracked files'}
foreach($rel in $tracked){[void]$protected.Add([IO.Path]::GetFullPath((Join-Path $root $rel)))}
$boards=@(Get-ChildItem -LiteralPath $root -Directory -Force | Where-Object {$_.Name -like 'OV5640*'})
foreach($board in $boards){
 foreach($pds in @(Get-ChildItem -LiteralPath $board.FullName -Filter '*.pds' -File)){
  [xml]$project=Get-Content -LiteralPath $pds.FullName -Raw -Encoding UTF8
  foreach($item in $project.SelectNodes('//*[@file]')){
   if($item.file){[void]$protected.Add([IO.Path]::GetFullPath((Join-Path $pds.DirectoryName $item.file)))}
  }
 }
}
$relative=@('build','logbackup','OV5640_DualView_100H/logbackup',
 'algorithom/closer2fpga/tests/build','algorithom/closer2fpga/exports')
$roots=@($relative | ForEach-Object {Join-Path $root $_} | Where-Object {Test-Path -LiteralPath $_})
# These directory names are compiler/cache outputs only within the C++ projects.
foreach($tree in @('algorithom','scripts','data/generators')){
 $base=Join-Path $root $tree
 $roots+=@(Get-ChildItem -LiteralPath $base -Directory -Force -Recurse |
  Where-Object {$_.Name -eq '__pycache__' -or
   ($tree -eq 'algorithom' -and $_.Name -in @('x64','x86','Debug','Release'))} |
  Select-Object -ExpandProperty FullName)
}
# Collapse nested targets so that each file is visited once.
$targets=@()
foreach($path in ($roots | Sort-Object -Unique | Sort-Object Length)){
 if(!@($targets | Where-Object {$path.StartsWith($_+'\',[StringComparison]::OrdinalIgnoreCase)}).Count){$targets+= $path}
}
$plans=@()
foreach($target in $targets){
 Assert-LocalPath $target
 $entries=@(Get-Item -LiteralPath $target -Force)+@(Get-ChildItem -LiteralPath $target -Force -Recurse)
 if(@($entries | Where-Object {$_.Attributes -band [IO.FileAttributes]::ReparsePoint}).Count){throw "Refusing linked directory: $target"}
 $files=@($entries | Where-Object {!$_.PSIsContainer})
 $keep=@($files | Where-Object {$protected.Contains($_.FullName) -or $_.Extension -in @('.sbit','.bit','.pof','.sof','.bin')})
 $deletable=@($files | Where-Object {!$protected.Contains($_.FullName) -and $_.Extension -notin @('.sbit','.bit','.pof','.sof','.bin')})
 $plans+= [pscustomobject]@{Path=$target;Entries=$entries;Files=$deletable;Keep=$keep.Count}
}
# Clear logs without deleting the PDS implementation database or JTAG bitstream.
$logs=@(Get-ChildItem -LiteralPath (Join-Path $root 'OV5640_DualView_100H') -File -Filter '*.log')
$logDir=Join-Path $root 'OV5640_DualView_100H/log'
if(Test-Path -LiteralPath $logDir){$logs+=@(Get-ChildItem -LiteralPath $logDir -File -Filter '*.log')}
foreach($file in $logs){Assert-LocalPath $file.FullName}
$removedFiles=0;$removedBytes=0L;$kept=0
foreach($plan in $plans){
 $kept+=$plan.Keep
 $bytes=($plan.Files | Measure-Object Length -Sum).Sum
 if($PSCmdlet.ShouldProcess($plan.Path,"Remove $($plan.Files.Count) generated files; preserve $($plan.Keep)")){
  if($plan.Keep -eq 0){Remove-Item -LiteralPath $plan.Path -Recurse -Force}
  else {
   foreach($file in $plan.Files){Remove-Item -LiteralPath $file.FullName -Force}
   foreach($dir in @($plan.Entries | Where-Object {$_.PSIsContainer} | Sort-Object { $_.FullName.Length } -Descending)){
    if(!@(Get-ChildItem -LiteralPath $dir.FullName -Force).Count){Remove-Item -LiteralPath $dir.FullName -Force}
   }
  }
  $removedFiles+=$plan.Files.Count;$removedBytes+=$bytes
 }
}
foreach($file in $logs){
 if(!$protected.Contains($file.FullName) -and $PSCmdlet.ShouldProcess($file.FullName,'Remove generated log')){
  Remove-Item -LiteralPath $file.FullName -Force
  $removedFiles++;$removedBytes+=$file.Length
 }
}
[pscustomobject]@{Files=$removedFiles;MiB=[math]::Round($removedBytes/1MB,2);ProtectedFiles=$kept}
