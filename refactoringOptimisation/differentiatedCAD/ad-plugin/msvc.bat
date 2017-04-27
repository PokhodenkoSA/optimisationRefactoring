@echo off

set BUILD_CONFIG=Debug
if not "%3" == "" set BUILD_CONFIG=%3

call %~dp0env.bat %1 %2 %3

echo Running Visual Studio...

if ["%VCVER%"] == ["vc10"] (
  start "%VS100COMNTOOLS%\..\IDE\devenv.exe" "msvc\ad-plugin.sln"
) else (
  echo "Unsupported version of MS Visual Studio"
  exit 1
)
