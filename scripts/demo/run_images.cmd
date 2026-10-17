@echo off
setlocal
if not defined VS_ROOT set "VS_ROOT=E:\vs"
if not defined OPENCV_ROOT set "OPENCV_ROOT=%VS_ROOT%\vcpkg\installed\x64-windows"
call "%VS_ROOT%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 2
set "OUT=%~dp0..\..\build\demo\images"
if not exist "%OUT%" mkdir "%OUT%"
pushd "%OUT%"
cl /nologo /std:c++20 /EHsc /O2 /fp:strict /MD /utf-8 /W3 /I"%OPENCV_ROOT%\include\opencv4" "%~dp0check_images.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\desktop\display.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\algo\calibrate.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\common\math3.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\algo\calibration\report.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\algo\calibration\lm.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\algo\calibration\residuals.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\algo\calibration\pose_init.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\algo\calibration\initialization.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\common\matrix.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\algo\undistort.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\algo\remap_table.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\algo\grayscale.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\algo\chessboard.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\algo\chessboard\validation.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\algo\chessboard\ordering.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\algo\chessboard\candidates.cpp" "%~dp0..\..\algorithom\closer2fpga\closer2fpga\algo\subpixel.cpp" /Fe:check_images.exe /link /LIBPATH:"%OPENCV_ROOT%\lib" opencv_core4.lib opencv_imgproc4.lib opencv_imgcodecs4.lib opencv_highgui4.lib
if errorlevel 1 (popd & exit /b 2)
set "PATH=%OPENCV_ROOT%\bin;%PATH%"
check_images.exe "%~dp0..\..\algorithom" "%OUT%"
if errorlevel 1 (popd & exit /b 3)
check_images.exe "%~dp0..\..\algorithom" "%OUT%\synthetic" synthetic
set "RESULT=%errorlevel%"
popd
exit /b %RESULT%
