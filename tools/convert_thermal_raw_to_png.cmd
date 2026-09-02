@echo off
setlocal
set "PPBNG_PYTHON=C:\Users\cairlab\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if not exist "%PPBNG_PYTHON%" (
  echo ERROR: bundled Python runtime not found: "%PPBNG_PYTHON%" 1>&2
  exit /b 2
)
"%PPBNG_PYTHON%" "%~dp0convert_thermal_raw_to_png.py" %*
exit /b %errorlevel%
