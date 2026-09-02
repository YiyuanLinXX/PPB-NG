@echo off
setlocal
if "%~1"=="" (
  echo Usage: tools\run_thermal_focus_test.cmd DATASET_NAME
  echo Example: tools\run_thermal_focus_test.cmd focus_trial_01
  exit /b 64
)
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_thermal_focus_test.ps1" "%~1"
exit /b %ERRORLEVEL%
