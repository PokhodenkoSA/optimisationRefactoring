@echo off

set VCVER=%1
set ARCH=%2
set CONFIG=%3

if "%VCVER%"  == "" set VCVER=vc10
if "%ARCH%"   == "" set ARCH=64
if "%CONFIG%" == "" set CONFIG=debug

cd %~dp0test

call %~dp0env.bat %VCVER% %ARCH% %CONFIG%
DRAWEXE.exe
cd ..
