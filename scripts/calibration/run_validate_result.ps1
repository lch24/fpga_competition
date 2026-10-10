param([string]$ModelSimBin=$env:MODELSIM_BIN)
$ErrorActionPreference='Stop'
if($ModelSimBin){$env:MODELSIM_BIN=$ModelSimBin}
& python (Join-Path $PSScriptRoot 'engine/check_validate.py')
if($LASTEXITCODE){throw 'validate regression failed'}
