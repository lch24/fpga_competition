param([string]$PdsBin=$env:PDS_SHELL)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot '../common/tools.ps1')
if(!$PdsBin){$PdsBin=(Get-Command pds_shell.exe -ErrorAction Stop).Source}
$root=Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$projectDir=Join-Path $root 'OV5640_DualView_100H'
$project=Join-Path $projectDir 'DualView_OV5640.pds'
$build=Join-Path $root 'build/system'
New-Item -ItemType Directory -Force $build | Out-Null
$before=New-Object System.Xml.XmlDocument
$before.PreserveWhitespace=$true
$before.Load($project)
$backup=$project+'.before_calibrated.bak'
if(!(Test-Path -LiteralPath $backup)){Copy-Item -LiteralPath $project -Destination $backup}
$constraintsHash=(Get-FileHash (Join-Path $projectDir 'DualView_OV5640.fdc')).Hash
$inputs=$before.SelectNodes('//task[@name="DESIGN_SET"]/action[@name="design"]/inputs/item[@type="FILE"]')
$excluded=Get-Content (Join-Path $root 'integration/board/excluded_sources.txt') | Where-Object {$_ -and ! $_.StartsWith('#')}
$common=Get-Content (Join-Path $root 'rtl/files.f') | Where-Object {$_ -and ! $_.StartsWith('+') -and $_ -notin $excluded}
$board=@('rtl/video/board/board_ms72xx_ctl.v','rtl/video/board/board_power_on_delay.v','rtl/top/calibrated_view_top.v')
$legacy=@('sync_vg','ms7200_ctl','ms7210_ctl','iic_dri','i2c_com','reg_config') | ForEach-Object {'rtl/video/board/'+$_+'.v'}
$all=@($common)+$board+$legacy
foreach($f in $all){if(!(Test-Path -LiteralPath (Join-Path $root $f))){throw "Missing source $f"}}
# PDS 2025.2 remove_design may report success without removing saved inputs.
# Prune only obsolete source references in XML, then let native PDS load/save it.
$keep=@($all | ForEach-Object {[IO.Path]::GetFullPath((Join-Path $root $_))})
foreach($item in $inputs){
 if([IO.Path]::GetFullPath((Join-Path $projectDir $item.file)) -notin $keep){
  [void]$item.ParentNode.RemoveChild($item)
 }
}
# Synchronize additions in XML as well: this PDS version can acknowledge
# add_design/save_project while leaving newly added file records unsaved.
$designInputs=$before.SelectSingleNode('//task[@name="DESIGN_SET"]/action[@name="design"]/inputs')
$present=@($designInputs.SelectNodes('item[@type="FILE"]') | ForEach-Object {[IO.Path]::GetFullPath((Join-Path $projectDir $_.file))})
foreach($f in $all){
 $absolute=[IO.Path]::GetFullPath((Join-Path $root $f))
 if($absolute -notin $present){
  $item=$before.CreateElement('item')
  $item.SetAttribute('type','FILE');$item.SetAttribute('file','../'+$f)
  $format=if($f.EndsWith('.sv')){'systemverilog'}else{'verilog'}
  $item.SetAttribute('format',$format);$item.SetAttribute('library','work')
  $item.SetAttribute('timespec',(Get-Item -LiteralPath $absolute).LastWriteTime.ToString('yyyy-MM-ddTHH:mm:ss'))
  $options=$before.CreateElement('options')
  foreach($entry in @(@('format',$format),@('library','work'))){
   $option=$before.CreateElement('option');$option.SetAttribute('name',$entry[0]);$option.SetAttribute('type','string');$option.SetAttribute('value',$entry[1]);[void]$options.AppendChild($option)
  }
  [void]$item.AppendChild($options);[void]$designInputs.AppendChild($item)
 }
}
foreach($item in $designInputs.SelectNodes('item[@type="FILE"]')){
 if(!$item.HasAttribute('timespec')){$item.SetAttribute('timespec',(Get-Date).ToString('yyyy-MM-ddTHH:mm:ss'))}
}
$before.Save($project)
function TclPath([string]$p){return '{'+$p.Replace('\','/')+'}'}
$tcl=@('open_project '+(TclPath $project))
$tcl+='set_option verilog_standard SystemVerilog [get_filesets design_1]'
# Add missing references; existing references are retained by PDS without duplication.
foreach($f in $all){
 $tcl+='add_design -verilog '+(TclPath (Join-Path $root $f))
}
$includes=@('rtl/include') | ForEach-Object {TclPath (Join-Path $root $_)}
$tcl+='set_option include_path [list '+($includes -join ' ')+'] [get_filesets design_1]'
$tcl+='set_option top_module calibrated_view_top [get_filesets design_1]'
$tcl+='save_project'
$tcl+='puts "PDS_ADD_SAVE_PASS"'
$tcl+='exit'
$script=Join-Path $build 'add_calibrated_pds.tcl'
[IO.File]::WriteAllLines($script,$tcl,[Text.UTF8Encoding]::new($false))
$romDir=Join-Path $projectDir 'data/rom'
New-Item -ItemType Directory -Force $romDir | Out-Null
Copy-Item (Join-Path $root 'data/rom/*.mem') -Destination $romDir -Force
Push-Location $build
try {
 $savedPreference=$ErrorActionPreference;$ErrorActionPreference='Continue'
 $output=& $PdsBin -file $script 2>&1
 $code=$LASTEXITCODE;$ErrorActionPreference=$savedPreference
 $output | Out-File (Join-Path $build 'pds_add.log')
 if($code -ne 0 -or ($output -join "`n") -notmatch 'PDS_ADD_SAVE_PASS' -or ($output -join "`n") -match '(?m)^E:'){throw "PDS add failed; backup: $backup; see build/system/pds_add.log"}
} finally {Pop-Location}
# Independent on-disk validation, followed by a second PDS process reopening it.
[xml]$after=[IO.File]::ReadAllText($project)
$design=$after.SelectSingleNode('//task[@name="DESIGN_SET"]')
if($design.SelectSingleNode('options/option[@name="top_module"]').value -ne 'calibrated_view_top'){throw 'Wrong saved top'}
$saved=@($design.SelectNodes('action[@name="design"]/inputs/item[@type="FILE"]'))
if($saved.Count -ne $all.Count){throw "Source count mismatch $($saved.Count) / $($all.Count)"}
$paths=@($saved | ForEach-Object {[IO.Path]::GetFullPath((Join-Path $projectDir $_.file))})
foreach($f in $all){if([IO.Path]::GetFullPath((Join-Path $root $f)) -notin $paths){throw "Source missing in PDS: $f"}}
if(@($paths | Sort-Object -Unique).Count -ne $paths.Count){throw 'Duplicate source references'}
foreach($p in $paths){if(!(Test-Path -LiteralPath $p)){throw "Invalid source path $p"}}
if($design.SelectSingleNode('options/option[@name="verilog_standard"]').value -ne 'SystemVerilog'){throw 'Incorrect HDL standard'}
if((Get-FileHash (Join-Path $projectDir 'DualView_OV5640.fdc')).Hash -ne $constraintsHash){throw 'FDC unexpectedly changed'}
$verify=@(('open_project '+(TclPath $project)),
 'get_option top_module [get_filesets design_1]',
 'report_option [get_filesets design_1]',
 'puts "PDS_REOPEN_VERIFY_PASS"','exit')
$verifyScript=Join-Path $build 'verify_calibrated_pds.tcl'
[IO.File]::WriteAllLines($verifyScript,$verify,[Text.UTF8Encoding]::new($false))
Push-Location $build
try {
 $savedPreference=$ErrorActionPreference;$ErrorActionPreference='Continue'
 $output=& $PdsBin -file $verifyScript 2>&1
 $code=$LASTEXITCODE;$ErrorActionPreference=$savedPreference
 $output | Out-File (Join-Path $build 'pds_reopen.log')
 $verifyText=$output -join "`n"
 if($code -ne 0 -or $verifyText -notmatch 'PDS_REOPEN_VERIFY_PASS' -or $verifyText -notmatch '(?m)^calibrated_view_top\s*$' -or $verifyText -match '(?m)^E:'){throw 'PDS reopen verification failed'}
} finally {Pop-Location}
$report=[ordered]@{project=$project;top='calibrated_view_top';sources=$saved.Count;duplicate_sources=0;missing_sources=0;backup=$backup;fdc_unchanged=$true;pds_reopen_pass=$true;compile_or_synthesis_run=$false}
$report | ConvertTo-Json | Set-Content (Join-Path $build 'pds_configuration.json') -Encoding UTF8
$report | ConvertTo-Json
