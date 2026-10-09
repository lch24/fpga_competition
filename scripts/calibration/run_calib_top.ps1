param([string]$ModelSimBin=$env:MODELSIM_BIN,[switch]$ControlOnly,[string]$ExportDirectory='')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot '../common/tools.ps1')
$ModelSimBin=Resolve-ToolDirectory $ModelSimBin 'vsim.exe' 'MODELSIM_BIN'
$ScriptDir=$PSScriptRoot
$RepoRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$BuildRoot=Join-Path $RepoRoot 'build/calibration'
New-Item -ItemType Directory -Force $BuildRoot | Out-Null

if($ControlOnly -and $ExportDirectory) { throw 'ControlOnly and ExportDirectory cannot be combined' }
if($ExportDirectory) { $ExportDirectory=(Resolve-Path -LiteralPath $ExportDirectory).Path }
Push-Location $BuildRoot
try {
 $buildName=if($ExportDirectory){'build_calib_real'}elseif($ControlOnly){'build_calib_top_control'}else{'build_calib_top'}
 if (!(Test-Path -LiteralPath $buildName)) { New-Item -ItemType Directory -Path $buildName | Out-Null }
 Push-Location (Join-Path $BuildRoot $buildName)
 try {
  if($ExportDirectory) {
   & node ../../../data/generators/calibration/calib_real_data.js prepare $ExportDirectory .
   if($LASTEXITCODE -ne 0) { throw 'Real export validation/preparation failed' }
  }
  $defines=@()
  if($ExportDirectory) {
   $metadata=Get-Content -LiteralPath (Join-Path $ExportDirectory calibration.json) -Raw -Encoding UTF8 | ConvertFrom-Json
   $defines=@("+define+PAR_VIEWS=$($metadata.views.Count)+PAR_BOARD_ROWS=$($metadata.rows)+PAR_BOARD_COLS=$($metadata.cols)+PAR_LM_MAX_ITERS=$($metadata.max_iterations_per_stage)+PAR_LM_MAX_TRIES=16")
  }
  if (!(Test-Path -LiteralPath work)) { & "$ModelSimBin\vlib.exe" work; if ($LASTEXITCODE -ne 0) { throw 'vlib failed' } }
  # Compile the exact public manifest, rewriting only its relative path prefix.
  $manifest=Get-Content ../../../rtl/files.f | ForEach-Object { if($_ -match '^\+incdir\+') { $_ -replace '^\+incdir\+','+incdir+../../../' } else { '../../../'+$_ } }
  Set-Content -LiteralPath rtl.f -Value $manifest -Encoding ASCII
  & "$ModelSimBin\vlog.exe" -sv -work work @defines -f rtl.f
  if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed' }
  & "$ModelSimBin\vlog.exe" -sv -work work @defines +incdir+../../../rtl/include ../../../tb/calibration/tb_calib_top.sv
  if ($LASTEXITCODE -ne 0) { throw 'TB compilation failed' }
  $control=if($ControlOnly){1}else{0}
  $realInput=if($ExportDirectory){1}else{0}
  # Full visibility disables many simulator optimizations. Full RTL needs only
  # TB counters for Tcl; Verilog hierarchical checks retain their own signals.
  $voptArgs=if($ControlOnly){'+acc'}else{'+acc=r+/tb_calib_top -O5'}
  # ModelSim 10.1c can emit a harmless FileWatch Tcl error on stderr at exit.
  # Preserve real simulator exit status and the explicit HDL/Tcl pass checks.
  $savedPreference=$ErrorActionPreference
  $ErrorActionPreference='Continue'
  $simOutput=Invoke-Simulator "$ModelSimBin\vsim.exe" -c "-voptargs=$voptArgs" -l simulation.log work.tb_calib_top "-gCONTROL_ONLY=$control" "-gREAL_INPUT=$realInput" -do ../../../scripts/calibration/run_calib_top.do 2>&1
  $simExit=$LASTEXITCODE
  $ErrorActionPreference=$savedPreference
  $simOutput | ForEach-Object { $_.ToString() }
  if($ExportDirectory) {
   & node ../../../data/generators/calibration/calib_real_data.js compare $ExportDirectory .
   if($LASTEXITCODE -ne 0) { throw 'Real image numerical comparison failed; see real_comparison.md' }
  }
  if ($simExit -ne 0) { throw 'calib_top simulation failed' }
  if (!(Select-String -LiteralPath simulation.log -SimpleMatch CALIB_TOP_PASS -Quiet)) { throw 'Missing pass marker' }
  $resultFile=if($ControlOnly){'calib_top_control_results.txt'}else{'calib_top_results.txt'}
  if (!$ControlOnly -and !$ExportDirectory) {
   # Independently check that the actual minimum among RTL candidates was
   # published. C++ candidates can be reordered by last-bit floating rounding.
   $bestCost=[double]::PositiveInfinity; $bestSeed=-1; $bestSteps=0; $totalSteps=0; $callCount=0
   $reportedSeed=-1; $reportedSteps=-1
   foreach($line in Get-Content -LiteralPath $resultFile) {
    if($line -match '^LM seed=(\d+) stage=(\d+) cycles=(\d+) cost=(\S+) converged=(\d+) accepted=(\d+) status=(\d+)$') {
     $seedNumber=[int]$Matches[1]; $stageNumber=[int]$Matches[2]
     $costValue=[double]::Parse($Matches[4],[Globalization.CultureInfo]::InvariantCulture)
     if([int]$Matches[7] -ne 0 -or $seedNumber -ne 2 -or $stageNumber -ne 2 -or $callCount -ne 0) { throw 'Unexpected full RTL stage sequence/status' }
     if($stageNumber -eq 0) { $totalSteps=0 }
     $totalSteps += [int]$Matches[6]; $callCount++
     if($stageNumber -eq 2 -and $costValue -lt $bestCost) { $bestCost=$costValue; $bestSeed=$seedNumber; $bestSteps=$totalSteps }
    }
    if($line -match '^case=0 .* status=0 seed=(\d+) steps=(\d+)$') { $reportedSeed=[int]$Matches[1]; $reportedSteps=[int]$Matches[2] }
   }
   if($callCount -ne 1 -or $reportedSeed -ne $bestSeed -or $reportedSteps -ne $bestSteps) { throw 'Published best seed/accepted count does not match RTL candidate history' }
   Write-Output "RTL_SELECTION_PASS seed=$bestSeed accepted=$bestSteps cost=$bestCost"
  }
  Get-Content -LiteralPath $resultFile
 } finally { Pop-Location }
} finally { Pop-Location }
