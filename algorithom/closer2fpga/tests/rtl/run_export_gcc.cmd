@echo off
rem ---------------------------------------------------------------------------
rem run_export_gcc.cmd - build and run the RTL vector export tool with MSYS2 g++
rem usage: tests\rtl\run_export_gcc.cmd
rem output: tests\build\vectors\  (binary vectors + manifests)
rem requires: D:\msys64\ucrt64\bin\g++.exe (any recent g++ works)
rem ---------------------------------------------------------------------------
setlocal
set "GXX=D:\msys64\ucrt64\bin\g++.exe"
if not exist "%GXX%" set "GXX=g++"

if not exist "%~dp0..\build" mkdir "%~dp0..\build"
if not exist "%~dp0..\build\raw" (
    powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0jpg_to_bgr.ps1" -Root "%~dp0..\.." -OutDir "%~dp0..\build\raw"
    if errorlevel 1 exit /b 2
)

pushd "%~dp0..\build"
"%GXX%" -std=c++20 -O2 -Wall -o export_vectors.exe "%~dp0export_vectors.cpp" "%~dp0..\..\closer2fpga\algo\shi_tomasi.cpp"
if errorlevel 1 exit /b 2
if not exist vectors mkdir vectors
export_vectors.exe "%~dp0..\build\raw" "%~dp0..\build\vectors"
set "result=%errorlevel%"
popd
exit /b %result%
