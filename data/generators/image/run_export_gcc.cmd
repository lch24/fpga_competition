@echo off
rem ---------------------------------------------------------------------------
rem run_export_gcc.cmd - build and run the RTL vector export tool with MSYS2 g++
rem usage: tests\rtl\run_export_gcc.cmd
rem output: tests\build\vectors\  (binary vectors + manifests)
rem Set GXX or add g++ to PATH.
rem ---------------------------------------------------------------------------
setlocal
if not defined GXX set "GXX=g++"
if not exist "%GXX%" set "GXX=g++"

if not exist "%~dp0..\..\..\build\image" mkdir "%~dp0..\..\..\build\image"
if not exist "%~dp0..\..\image\raw" (
    powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0jpg_to_bgr.ps1" -Root "%~dp0..\..\..\algorithom\closer2fpga" -OutDir "%~dp0..\..\image\raw"
    if errorlevel 1 exit /b 2
)

pushd "%~dp0..\..\..\build\image"
"%GXX%" -std=c++20 -O2 -Wall -o export_vectors.exe "%~dp0export_vectors.cpp" "%~dp0..\..\..\algorithom\closer2fpga\closer2fpga\algo\shi_tomasi.cpp"
if errorlevel 1 exit /b 2
if not exist "%~dp0..\..\image" mkdir "%~dp0..\..\image"
export_vectors.exe "%~dp0..\..\image\raw" "%~dp0..\..\image"
set "result=%errorlevel%"
popd
exit /b %result%
