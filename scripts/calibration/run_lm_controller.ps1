param([string]$ModelSimBin=$env:MODELSIM_BIN)
$ErrorActionPreference='Stop'
if($ModelSimBin){$env:MODELSIM_BIN=$ModelSimBin}
& python (Join-Path $PSScriptRoot 'engine/check_lm.py')
if($LASTEXITCODE){throw 'lm regression failed'}
