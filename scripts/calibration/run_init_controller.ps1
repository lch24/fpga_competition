param([string]$ModelSimBin=$env:MODELSIM_BIN)
$ErrorActionPreference='Stop'
if($ModelSimBin){$env:MODELSIM_BIN=$ModelSimBin}
& python (Join-Path $PSScriptRoot 'engine/check_init.py')
if($LASTEXITCODE){throw 'init regression failed'}
