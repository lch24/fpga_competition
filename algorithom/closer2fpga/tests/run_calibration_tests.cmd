@echo off
setlocal
if not defined VS_ROOT set "VS_ROOT=E:\vs"
if not defined OPENCV_ROOT set "OPENCV_ROOT=%VS_ROOT%\vcpkg\installed\x64-windows"
call "%VS_ROOT%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 2
if not exist "%~dp0build" mkdir "%~dp0build"
pushd "%~dp0build"
cl /nologo /std:c++20 /EHsc /O2 /MD /utf-8 /W3 /I"%OPENCV_ROOT%\include\opencv4" "%~dp0calibration_regression.cpp" "%~dp0..\closer2fpga\algo\calibrate.cpp" "%~dp0..\closer2fpga\common\matrix.cpp" "%~dp0..\closer2fpga\algo\undistort.cpp" "%~dp0..\closer2fpga\algo\chessboard.cpp" "%~dp0..\closer2fpga\algo\shi_tomasi.cpp" "%~dp0..\closer2fpga\algo\subpixel.cpp" /Fe:calibration_regression.exe /link /LIBPATH:"%OPENCV_ROOT%\lib" opencv_core4.lib opencv_imgproc4.lib opencv_imgcodecs4.lib opencv_calib3d4.lib
if errorlevel 1 exit /b 2
set "PATH=%OPENCV_ROOT%\bin;%PATH%"
calibration_regression.exe "%~dp0..\.."
set "test_result=%errorlevel%"
if not "%test_result%"=="0" exit /b %test_result%
cl /nologo /std:c++20 /EHsc /O2 /MT /utf-8 /W3 "%~dp0pure_core_smoke.cpp" "%~dp0..\closer2fpga\algo\calibrate.cpp" "%~dp0..\closer2fpga\common\matrix.cpp" "%~dp0..\closer2fpga\algo\undistort.cpp" /Fe:pure_core_smoke.exe
if errorlevel 1 exit /b 2
pure_core_smoke.exe
set "test_result=%errorlevel%"
popd
exit /b %test_result%
