@echo off
setlocal
if not defined VS_ROOT set "VS_ROOT=E:\vs"
if not defined OPENCV_ROOT set "OPENCV_ROOT=%VS_ROOT%\vcpkg\installed\x64-windows"
call "%VS_ROOT%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 2
if not exist "%~dp0build" mkdir "%~dp0build"
pushd "%~dp0build"
cl /nologo /std:c++20 /EHsc /O2 /MD /utf-8 /W3 /I"%OPENCV_ROOT%\include\opencv4" "%~dp0..\closer2fpga\main.cpp" "%~dp0..\closer2fpga\desktop\display.cpp" "%~dp0..\closer2fpga\algo\calibrate.cpp" "%~dp0..\closer2fpga\common\symmetric_eigen.cpp" "%~dp0..\closer2fpga\common\math3.cpp" "%~dp0..\closer2fpga\algo\calibration\report.cpp" "%~dp0..\closer2fpga\algo\calibration\lm.cpp" "%~dp0..\closer2fpga\algo\calibration\residuals.cpp" "%~dp0..\closer2fpga\algo\calibration\pose_init.cpp" "%~dp0..\closer2fpga\algo\calibration\initialization.cpp" "%~dp0..\closer2fpga\common\matrix.cpp" "%~dp0..\closer2fpga\algo\undistort.cpp" "%~dp0..\closer2fpga\algo\remap_table.cpp" "%~dp0..\closer2fpga\algo\grayscale.cpp" "%~dp0..\closer2fpga\algo\chessboard.cpp" "%~dp0..\closer2fpga\algo\chessboard\validation.cpp" "%~dp0..\closer2fpga\algo\chessboard\ordering.cpp" "%~dp0..\closer2fpga\algo\chessboard\candidates.cpp" "%~dp0..\closer2fpga\algo\shi_tomasi.cpp" "%~dp0..\closer2fpga\algo\subpixel.cpp" /Fe:closer2fpga_demo.exe /link /LIBPATH:"%OPENCV_ROOT%\lib" opencv_core4.lib opencv_imgproc4.lib opencv_imgcodecs4.lib opencv_highgui4.lib
set "build_result=%errorlevel%"
popd
exit /b %build_result%
