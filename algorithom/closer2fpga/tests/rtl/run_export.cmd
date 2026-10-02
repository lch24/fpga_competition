@echo off
rem ---------------------------------------------------------------------------
rem run_export.cmd - build and run the RTL vector export tool with MSVC (cl)
rem usage: tests\rtl\run_export.cmd
rem output: tests\build\vectors\  (binary vectors + manifests)
rem No OpenCV dependency: JPEG decoding uses jpg_to_bgr.ps1 (.NET System.Drawing)
rem ---------------------------------------------------------------------------
setlocal
if not defined VS_ROOT set "VS_ROOT=E:\vs"
call "%VS_ROOT%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
where cl >nul 2>&1
if errorlevel 1 (
    echo [run_export] MSVC cl not found, trying MSYS2 g++ ...
    call "%~dp0run_export_gcc.cmd"
    exit /b %errorlevel%
)

if not exist "%~dp0..\build" mkdir "%~dp0..\build"
if not exist "%~dp0..\build\raw" (
    powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0jpg_to_bgr.ps1" -Root "%~dp0..\.." -OutDir "%~dp0..\build\raw"
    if errorlevel 1 exit /b 2
)

pushd "%~dp0..\build"
cl /nologo /std:c++20 /EHsc /O2 /MD /utf-8 /W3 "%~dp0export_vectors.cpp" "%~dp0..\..\closer2fpga\algo\shi_tomasi.cpp" /Fe:export_vectors.exe
if errorlevel 1 exit /b 2
if not exist vectors mkdir vectors
export_vectors.exe "%~dp0..\build\raw" "%~dp0..\build\vectors"
set "result=%errorlevel%"
popd
exit /b %result%
