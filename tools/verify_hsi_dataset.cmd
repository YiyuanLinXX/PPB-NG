@echo off
setlocal

if "%~1"=="" (
  echo Usage: tools\verify_hsi_dataset.cmd ^<session-directory^> [--hsi-only fx10e^|swir]
  exit /b 2
)

echo Note: verify_hsi_dataset.cmd is retained as a compatibility alias for verify_dataset.cmd.
call "%~dp0verify_dataset.cmd" %*
exit /b %errorlevel%
