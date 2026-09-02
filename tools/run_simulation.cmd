@echo off
setlocal

set "PPBNG_VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat"
set "PPBNG_ROS_SETUP=C:\pixi_ws\ros2-windows\local_setup.bat"
set "PPBNG_PIXI_RUNTIME=C:\pixi_ws\.pixi\envs\default\Library\bin"
set "PPBNG_WORKSPACE_SETUP=%~dp0..\install\local_setup.bat"

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
if not exist "%PPBNG_WORKSPACE_SETUP%" (
  echo ERROR: Workspace has not been built. Run tools\build.cmd first. 1>&2
  exit /b 2
)

call "%PPBNG_VCVARS%"
if errorlevel 1 exit /b %errorlevel%
call "%PPBNG_ROS_SETUP%"
if errorlevel 1 exit /b %errorlevel%
call "%PPBNG_WORKSPACE_SETUP%"
if errorlevel 1 exit /b %errorlevel%
set "PATH=%PPBNG_PIXI_RUNTIME%;%PATH%"

echo PPBNG simulation mode: no hardware backend is loaded.
ros2 launch ppbng_bringup simulation.launch.py %*
exit /b %errorlevel%
