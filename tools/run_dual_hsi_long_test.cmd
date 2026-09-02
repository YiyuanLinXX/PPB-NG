@echo off
setlocal

if "%~1"=="" (
  echo Usage: tools\run_dual_hsi_long_test.cmd DATASET_NAME [DURATION_MINUTES] [both^|fx10e^|swir] [-Unattended]
  exit /b 2
)

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_dual_hsi_long_test.ps1" %*
exit /b %errorlevel%
