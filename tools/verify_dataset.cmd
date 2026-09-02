@echo off
setlocal

if "%~1"=="" (
  echo Usage: tools\verify_dataset.cmd ^<session-directory^>
  exit /b 2
)

call "%~dp0ros2.cmd" run ppbng_hsi ppbng_verify_dataset %*
exit /b %errorlevel%
