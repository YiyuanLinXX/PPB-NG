@echo off
setlocal

set "PPBNG_VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat"
set "PPBNG_ROS_SETUP=C:\pixi_ws\ros2-windows\local_setup.bat"
set "PPBNG_PIXI_RUNTIME=C:\pixi_ws\.pixi\envs\default\Library\bin"
set "PPBNG_SOAK_EXE=%~dp0..\build\ppbng_sim\Release\ppbng_sim_soak.exe"

if not exist "%PPBNG_VCVARS%" (
  echo ERROR: Visual Studio x64 environment script not found: "%PPBNG_VCVARS%" 1>&2
  exit /b 2
)
if not exist "%PPBNG_ROS_SETUP%" (
  echo ERROR: ROS 2 environment script not found: "%PPBNG_ROS_SETUP%" 1>&2
  exit /b 2
)
if not exist "%PPBNG_PIXI_RUNTIME%\spdlog.dll" (
  echo ERROR: Pixi ROS runtime DLL directory is incomplete: "%PPBNG_PIXI_RUNTIME%" 1>&2
  exit /b 2
)
if not exist "%PPBNG_SOAK_EXE%" (
  echo ERROR: Soak executable is missing. Build the workspace first. 1>&2
  exit /b 2
)

call "%PPBNG_VCVARS%"
if errorlevel 1 exit /b %errorlevel%
call "%PPBNG_ROS_SETUP%"
if errorlevel 1 exit /b %errorlevel%
set "PATH=%PPBNG_PIXI_RUNTIME%;%PATH%"

set "PPBNG_SOAK_SECONDS=%~1"
if "%PPBNG_SOAK_SECONDS%"=="" set "PPBNG_SOAK_SECONDS=7200"
echo PPBNG virtual soak: %PPBNG_SOAK_SECONDS% seconds; no hardware backend is loaded.
"%PPBNG_SOAK_EXE%" "%PPBNG_SOAK_SECONDS%"
exit /b %errorlevel%
