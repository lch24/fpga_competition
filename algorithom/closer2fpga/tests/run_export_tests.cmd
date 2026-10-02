@echo off
setlocal
if not defined VS_ROOT set "VS_ROOT=E:\vs"
call "%VS_ROOT%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 2
if not exist "%~dp0build" mkdir "%~dp0build"
pushd "%~dp0build"
cl /nologo /std:c++20 /EHsc /O2 /MT /utf-8 /W3 "%~dp0export_regression.cpp" /Fe:export_regression.exe
if errorlevel 1 exit /b 2
export_regression.exe
if errorlevel 1 exit /b 1
node "%~dp0verify_export.js"
set "test_result=%errorlevel%"
popd
exit /b %test_result%
