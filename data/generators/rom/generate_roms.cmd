@echo off
setlocal
cd /d "%~dp0..\..\.."
if not exist build\system mkdir build\system
if not exist data\rom mkdir data\rom
rem Use VS_ROOT or a Visual Studio developer command prompt.
if defined VS_ROOT call "%VS_ROOT%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cl /nologo /EHsc /O2 /fp:strict /utf-8 data\generators\rom\generate_roms.cpp /Fe:build\system\generate_roms.exe /Fo:build\system\generate_roms.obj
if errorlevel 1 exit /b 1
build\system\generate_roms.exe
