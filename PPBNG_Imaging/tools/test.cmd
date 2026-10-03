@echo off
setlocal

rem Use batch dependency hooks consistently, even when invoked from PowerShell.
rem This avoids unsigned generated PS1 hooks being rejected by Windows policy.
if defined COLCON_EXTENSION_BLOCKLIST (
  set "COLCON_EXTENSION_BLOCKLIST=%COLCON_EXTENSION_BLOCKLIST%;colcon_core.shell.powershell"
) else (
  set "COLCON_EXTENSION_BLOCKLIST=colcon_core.shell.powershell"
)

set "PPBNG_VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat"
set "PPBNG_ROS_SETUP=C:\pixi_ws\ros2-windows\local_setup.bat"
set "PPBNG_PIXI_SCRIPTS=C:\pixi_ws\.pixi\envs\default\Scripts"

if not exist "%PPBNG_VCVARS%" (
  echo ERROR: Visual Studio x64 environment script not found: "%PPBNG_VCVARS%" 1>&2
  exit /b 2
)
if not exist "%PPBNG_ROS_SETUP%" (
  echo ERROR: ROS 2 environment script not found: "%PPBNG_ROS_SETUP%" 1>&2
  exit /b 2
)

call "%PPBNG_VCVARS%"
if errorlevel 1 exit /b %errorlevel%

call "%PPBNG_ROS_SETUP%"
if errorlevel 1 exit /b %errorlevel%
if not exist "%PPBNG_PIXI_SCRIPTS%\colcon.exe" (
  echo ERROR: colcon not found: "%PPBNG_PIXI_SCRIPTS%\colcon.exe" 1>&2
  exit /b 2
)
set "PATH=%PPBNG_PIXI_SCRIPTS%;%PATH%"

cd /d "%~dp0.."
if exist "%~dp0..\install\local_setup.bat" call "%~dp0..\install\local_setup.bat"

colcon test --event-handlers console_direct+ %*
if errorlevel 1 exit /b %errorlevel%

colcon test-result --verbose
exit /b %errorlevel%
