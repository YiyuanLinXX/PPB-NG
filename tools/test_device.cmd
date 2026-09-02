@echo off
setlocal

if "%~1"=="" goto usage
if not "%~2"=="" (
  echo ERROR: test_device accepts exactly one module name. 1>&2
  goto usage_error
)

set "PPBNG_MODULE=%~1"
set "PPBNG_PACKAGE="

if /i "%PPBNG_MODULE%"=="core" set "PPBNG_PACKAGE=ppbng_core"
if /i "%PPBNG_MODULE%"=="storage" set "PPBNG_PACKAGE=ppbng_storage"
if /i "%PPBNG_MODULE%"=="orchestrator" set "PPBNG_PACKAGE=ppbng_orchestrator"
if /i "%PPBNG_MODULE%"=="timing" set "PPBNG_PACKAGE=ppbng_timing"
if /i "%PPBNG_MODULE%"=="gnss" set "PPBNG_PACKAGE=ppbng_gnss"
if /i "%PPBNG_MODULE%"=="rsm400" set "PPBNG_PACKAGE=ppbng_rsm400"
if /i "%PPBNG_MODULE%"=="rgb" set "PPBNG_PACKAGE=ppbng_rgb"
if /i "%PPBNG_MODULE%"=="thermal" set "PPBNG_PACKAGE=ppbng_thermal"
if /i "%PPBNG_MODULE%"=="hsi" set "PPBNG_PACKAGE=ppbng_hsi"
if /i "%PPBNG_MODULE%"=="fx10e" set "PPBNG_PACKAGE=ppbng_hsi"
if /i "%PPBNG_MODULE%"=="swir" set "PPBNG_PACKAGE=ppbng_hsi"
if /i "%PPBNG_MODULE%"=="integration" set "PPBNG_PACKAGE=ppbng_sim"
if /i "%PPBNG_MODULE%"=="bringup" set "PPBNG_PACKAGE=ppbng_bringup"

if not defined PPBNG_PACKAGE (
  echo ERROR: unknown module "%PPBNG_MODULE%". 1>&2
  goto usage_error
)

echo PPBNG hardware-free module test: %PPBNG_MODULE% ^(%PPBNG_PACKAGE%^)
echo This command does not enumerate, open, configure, or acquire from hardware.
call "%~dp0test.cmd" --packages-select "%PPBNG_PACKAGE%"
exit /b %errorlevel%

:usage
echo Usage: tools\test_device.cmd MODULE
echo Modules: core storage orchestrator timing gnss rsm400 rgb thermal hsi fx10e swir integration bringup
exit /b 0

:usage_error
echo Usage: tools\test_device.cmd MODULE 1>&2
echo Modules: core storage orchestrator timing gnss rsm400 rgb thermal hsi fx10e swir integration bringup 1>&2
exit /b 2
