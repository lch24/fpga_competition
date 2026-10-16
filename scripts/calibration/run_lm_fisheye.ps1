# Fisheye LM case: synthetic corners from an independent model, expectation from
# an independent solve. Requires ModelSim on PATH or MODELSIM_BIN.
param([string]$ModelSimBin=$env:MODELSIM_BIN)
$ErrorActionPreference='Stop'
if($ModelSimBin){$env:MODELSIM_BIN=$ModelSimBin}
& python (Join-Path $PSScriptRoot 'engine/generate_lm_fisheye.py')
if($LASTEXITCODE){throw 'Could not generate the fisheye fixture'}
& python (Join-Path $PSScriptRoot 'engine/check_lm_fisheye.py')
if($LASTEXITCODE){throw 'Fisheye LM check failed'}
