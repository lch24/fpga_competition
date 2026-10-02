param([string]$ModelSimBin='E:\pangu\Modelsim10.1c\win64')
$ErrorActionPreference='Stop'
$build=Join-Path $PSScriptRoot build_config_invalid
if(!(Test-Path -LiteralPath $build)){New-Item -ItemType Directory -Path $build | Out-Null}
Push-Location $build
try {
 if(!(Test-Path work)){& "$ModelSimBin\vlib.exe" work;if($LASTEXITCODE -ne 0){throw 'vlib failed'}}
 $manifest=Get-Content ../../files.f | ForEach-Object {if($_ -match '^\+incdir\+'){$_ -replace '^\+incdir\+','+incdir+../../'}else{'../../'+$_}}
 Set-Content rtl.f $manifest -Encoding ASCII
 $cases=@('PAR_VIEWS=2','PAR_VIEWS=17','PAR_BOARD_ROWS=1','PAR_BOARD_COLS=1','PAR_BOARD_ROWS=17+PAR_BOARD_COLS=16','PAR_LM_MAX_ITERS=0','PAR_LM_MAX_ITERS=256','PAR_LM_MAX_TRIES=0','PAR_LM_MAX_TRIES=257')
 foreach($config in $cases){
  & "$ModelSimBin\vlog.exe" -work work "+define+$config" -f rtl.f | Out-Null
  if($LASTEXITCODE -ne 0){throw "Unexpected parse failure for $config"}
  $result=& "$ModelSimBin\vopt.exe" work.calib_top -o invalid_top
  if($LASTEXITCODE -eq 0 -or !($result -match 'CALIB_CONFIGURATION_OUT_OF_SUPPORTED_RANGE')){throw "Missing expected configuration guard for $config"}
  Write-Output "CONFIG_REJECT_PASS $config"
 }
 Write-Output "CONFIG_LIMITS_PASS cases=$($cases.Count)"
} finally {Pop-Location}
