param([string]$ModelSimBin=$env:MODELSIM_BIN)
$ErrorActionPreference='Stop'
if(!$ModelSimBin){$ModelSimBin=Split-Path (Get-Command vsim.exe -ErrorAction Stop).Source}
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$build=Join-Path $root 'build/camera_diagnosis'
New-Item -ItemType Directory -Force $build | Out-Null
$sources=@('rtl/video/stream/rgb565_format.v','rtl/video/dma/video_capture_hmic.v',
 'rtl/common/reset_sync.v','rtl/video/board/i2c_com.v','rtl/video/board/reg_config.v','rtl/video/board/board_power_on_delay.v',
 'rtl/common/async_fifo.v','rtl/common/async_bram_fifo.v',
 'tb/video/tb_camera_unpack.sv','tb/video/tb_camera_capture_chain.sv','tb/video/tb_camera_crop.sv',
 'tb/video/tb_camera_integrity.sv','tb/video/tb_camera_config.sv','tb/video/tb_camera_power.sv','tb/video/tb_reg_config_clock.sv','tb/system/tb_video_capture.sv')
Push-Location $build
try {
 if(!(Test-Path work)){& (Join-Path $ModelSimBin 'vlib.exe') work}
 & (Join-Path $ModelSimBin 'vlog.exe') @($sources|ForEach-Object{Join-Path $root $_}) *> compile.log
 if($LASTEXITCODE -ne 0){throw 'Compile failed: build/camera_diagnosis/compile.log'}
 foreach($test in @('tb_camera_unpack','tb_camera_capture_chain','tb_camera_crop','tb_camera_integrity','tb_video_capture','tb_camera_config','tb_camera_power','tb_reg_config_clock')) {
  & (Join-Path $ModelSimBin 'vsim.exe') -c -novopt -l "$test.log" $test -do 'run -all; quit -f' *> "$test.console.log"
  $log=Get-Content "$test.log" -Raw
  if($LASTEXITCODE -ne 0 -or $log -notmatch 'PASS ' -or $log -match '\*\* (Fatal|Error):') {throw "FAIL $test; see build/camera_diagnosis/$test.log"}
  Write-Host "PASS $test"
 }
} finally {Pop-Location}
