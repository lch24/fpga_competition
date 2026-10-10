param([string]$PdsShell=$env:PDS_SHELL,[string]$Python='python')
$ErrorActionPreference='Stop'
if(!$PdsShell){$PdsShell='pds_shell.exe'}
$RepoRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../..'))
# Isolated copies, process-tree memory cap and timeout are enforced by the
# shared runner. This never changes or starts the board's saved PDS project.
& $Python (Join-Path $RepoRoot 'scripts/build/check_partition_synthesis.py') --pds $PdsShell --tops calib_datapath probe_fp_basic --constraints (Join-Path $PSScriptRoot 'probe_40m.fdc') --memory-gib 6 --timeout 300
if($LASTEXITCODE){throw 'isolated resource synthesis failed'}
