@echo off
setlocal
if not defined VS_ROOT set "VS_ROOT=E:\vs"
call "%VS_ROOT%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 2
set "SRC=%~dp0..\closer2fpga"
set "OUT=%~dp0..\..\..\build\cpp_simplification"
set "REPORT=%~dp0..\..\..\data\reports\calibration_simplification"
set "INPUT=%~dp0..\..\..\data\real\run_1790731382958_0"
if not "%~1"=="" set "INPUT=%~f1"
if not "%~2"=="" set "REPORT=%~f2"
if not exist "%OUT%" mkdir "%OUT%"
pushd "%OUT%"
cl /nologo /std:c++20 /EHsc /O2 /MT /utf-8 /W4 "%~dp0recommended_regression.cpp" "%SRC%\algo\calibrate.cpp" "%SRC%\common\math3.cpp" "%SRC%\common\matrix.cpp" "%SRC%\algo\calibration\report.cpp" "%SRC%\algo\calibration\lm.cpp" "%SRC%\algo\calibration\residuals.cpp" "%SRC%\algo\calibration\pose_init.cpp" "%SRC%\algo\calibration\initialization.cpp" /Fe:recommended_regression.exe
if errorlevel 1 (popd & exit /b 2)
recommended_regression.exe "%INPUT%" "%REPORT%" "%~dp0..\..\..\data\fixtures\calibration\recommended.csv"
set "RESULT=%errorlevel%"
popd
exit /b %RESULT%
