@echo off
setlocal
cd /d "%~dp0..\.."
if not exist integration\build mkdir integration\build
if not exist integration\rom mkdir integration\rom
if not defined VS_ROOT set "VS_ROOT=E:\vs"
call "%VS_ROOT%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cl /nologo /EHsc /O2 /fp:strict /utf-8 integration\tools\generate_roms.cpp /Fe:integration\build\generate_roms.exe /Fo:integration\build\generate_roms.obj
if errorlevel 1 exit /b 1
integration\build\generate_roms.exe
