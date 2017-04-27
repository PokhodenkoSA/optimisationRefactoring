@echo off

if exist %~dp0custom.bat call %~dp0custom.bat

if /I "%2" == "win32" set ARCH=32
if /I "%2" == "win64" set ARCH=64

if "%VCVER%" == "" set VCVER=vc10
if "%ARCH%"  == "" set ARCH=64

set CASDEB=d
set BUILD_CONFIG=debug
if not "%3" == "" (
    if /I "%3" == "Debug" (
        set CASDEB=d
        set BUILD_CONFIG=debug
    ) else if /I "%3" == "Release" (
        set CASDEB=
        set BUILD_CONFIG=release
    ) else (
        echo Error: third argument ^(%3^) should specify build mode,
        echo one of: Debug or Release
        exit
    )
)

set DEB=%CASDEB%

rem ===========================================================================
rem Setting AD Plugin environment 
rem ===========================================================================

if "%AD_ROOT%"         == "" set "AD_ROOT=%~dp0"
if "%AD_RESOURCES%"    == "" set "AD_RESOURCES=%AD_ROOT%\res"
if "%AD_LIB%"          == "" set "AD_LIB=%AD_ROOT%\win%ARCH%\%VCVER%\lib%DEB%"
if "%AD_BIN%"          == "" set "AD_BIN=%AD_ROOT%\win%ARCH%\%VCVER%\bin%DEB%"
if "%AD_INC%"          == "" set "AD_INC=%AD_ROOT%\include"
if "%AD_TEST%"         == "" set "AD_TEST=%AD_ROOT%\test"
if "%AD_TEST_RESULTS%" == "" set "AD_TEST_RESULTS=%AD_ROOT%\test"
if "%AD_TEST_DATA%"    == "" set "AD_TEST_DATA=%AD_ROOT%\..\data"
if "%AD_PRODUCTS%"     == "" set "AD_PRODUCTS=%AD_ROOT%\.."

rem ===========================================================================
rem Setting OCCT environment 
rem ===========================================================================

if "%PRODUCTS_PATH%" == "" set "PRODUCTS_PATH=%AD_PRODUCTS%"
if "%CASROOT%"       == "" set "CASROOT=%AD_PRODUCTS%\occt"

set "CAS_INC=%CASROOT%\inc"
set "CAS_LIB32=%CASROOT%\win32\%VCVER%\lib%DEB%"
set "CAS_BIN32=%CASROOT%\win32\%VCVER%\bin%DEB%"
set "CAS_LIB64=%CASROOT%\win64\%VCVER%\lib%DEB%"
set "CAS_BIN64=%CASROOT%\win64\%VCVER%\bin%DEB%"

echo ...
echo ARCH         = %ARCH%
echo VCVER        = %VCVER%
echo BUILD_CONFIG = %BUILD_CONFIG%
echo SPE_ROOT     = %SPE_ROOT%
echo SPE_PRODUCTS = %SPE_PRODUCTS%
echo CASROOT      = %CASROOT%
echo CASDEB       = %CASDEB%
echo CAS_BIN64    = %CAS_BIN64%
echo ...

rem Call OCCT env.bat -- all 3-rd party binaries will be added to PATH
call %CASROOT%\env.bat %VCVER% win%ARCH% %BUILD_CONFIG%

rem ===========================================================================
rem Setting environment for ADOL-C
rem ===========================================================================

if "%ADOLC_ROOT%" == "" set "ADOLC_ROOT=%AD_PRODUCTS%\adolc_base"
set "ADOLC_INC=%ADOLC_ROOT%\include"
set "ADOLC_LIB=%ADOLC_ROOT%\lib64"

rem ===========================================================================
rem Adjust PATH variable
rem ===========================================================================

set "PATH=%AD_BIN%;%CAS_BIN64%;%PATH%"

rem ===========================================================================
rem Customize OCCT environment variables
rem ===========================================================================

set "CSF_PluginDefaults=%AD_RESOURCES%"
set "CSF_ResourcesDefaults=%AD_RESOURCES%"
set "CSF_ADDrawDefaults=%AD_RESOURCES%"
rem set "CSF_TestScriptsDumpingPath=%AD_TEST%\dumping"
set "CSF_TestDataPath=%AD_TEST_DATA%"
