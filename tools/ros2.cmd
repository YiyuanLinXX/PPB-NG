@echo off
setlocal

set "PPBNG_VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat"
set "PPBNG_ROS_SETUP=C:\pixi_ws\ros2-windows\local_setup.bat"
set "PPBNG_ROS_SCRIPTS=C:\pixi_ws\ros2-windows\Scripts"
set "PPBNG_PIXI_RUNTIME=C:\pixi_ws\.pixi\envs\default\Library\bin"
set "PPBNG_WORKSPACE_SETUP=%~dp0..\install\local_setup.bat"

rem PPB-NG has been qualified with eProsima Fast DDS. Pin the middleware so an
rem unrelated machine/user environment variable cannot silently select the
rem optional RTI Connext backend.
set "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"

rem Windows without Developer Mode cannot create ROS launch's convenience
rem 'latest' log-directory symlink. Log directories are still created normally;
rem suppress only that known Python warning, while preserving every other one.
if defined PYTHONWARNINGS (
  set "PYTHONWARNINGS=ignore:Cannot create a symlink to latest log directory:UserWarning,%PYTHONWARNINGS%"
) else (
  set "PYTHONWARNINGS=ignore:Cannot create a symlink to latest log directory:UserWarning"
)

if not exist "%PPBNG_VCVARS%" exit /b 2
if not exist "%PPBNG_ROS_SETUP%" exit /b 2
if not exist "%PPBNG_ROS_SCRIPTS%\ros2.exe" exit /b 2
if not exist "%PPBNG_WORKSPACE_SETUP%" exit /b 2

call "%PPBNG_VCVARS%" >nul
if errorlevel 1 exit /b %errorlevel%

rem This ROS distribution was packaged with an RTI Connext environment hook,
rem but the proprietary RTI runtime is intentionally not installed. Capture the
rem setup stderr and remove only that exact optional-backend warning. Any other
rem setup diagnostic is forwarded unchanged and any nonzero setup exit remains
rem fatal.
set "PPBNG_SETUP_STDERR=%TEMP%\ppbng_ros_setup_%RANDOM%_%RANDOM%.stderr"
call "%PPBNG_ROS_SETUP%" 2>"%PPBNG_SETUP_STDERR%"
set "PPBNG_SETUP_EXIT=%errorlevel%"
if exist "%PPBNG_SETUP_STDERR%" (
  %SystemRoot%\System32\findstr.exe /V /L /C:"[rti_connext_dds_cmake_module][warning] RTI Connext DDS environment script not found" "%PPBNG_SETUP_STDERR%" 1>&2
  del /Q "%PPBNG_SETUP_STDERR%" >nul 2>&1
)
if not "%PPBNG_SETUP_EXIT%"=="0" exit /b %PPBNG_SETUP_EXIT%
call "%PPBNG_WORKSPACE_SETUP%"
if errorlevel 1 exit /b %errorlevel%
set "PATH=%PPBNG_ROS_SCRIPTS%;%PPBNG_PIXI_RUNTIME%;%PATH%"

ros2 %*
exit /b %errorlevel%
