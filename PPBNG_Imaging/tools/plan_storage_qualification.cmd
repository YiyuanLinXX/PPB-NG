@echo off
setlocal
set "PPBNG_PYTHON=C:\pixi_ws\.pixi\envs\default\python.exe"
if not exist "%PPBNG_PYTHON%" (
  echo ERROR: Python environment not found: "%PPBNG_PYTHON%" 1>&2
  exit /b 2
)
"%PPBNG_PYTHON%" "%~dp0storage_qualification_plan.py" %*
exit /b %errorlevel%
