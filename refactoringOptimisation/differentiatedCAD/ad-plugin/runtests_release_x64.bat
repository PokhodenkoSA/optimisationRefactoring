@echo off

call %~dp0env.bat vc12 win64 release

rem replace \ by / in SPE_TEST_RESULTS variable
set OUTDIR=%SPE_TEST_RESULTS:\=/%

echo.
echo Tests output dir: %OUTDIR%
echo.

cd "%OUTDIR%"

call DRAWEXE.exe -c testgrid -overwrite -parallel 4
pause > nul
