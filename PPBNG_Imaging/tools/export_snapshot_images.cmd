@echo off
setlocal
rem Share the stable offline environment with the HSI preview command.
if not defined PPBNG_PYTHON set "PPBNG_PYTHON=%~dp0..\.venv-export\Scripts\python.exe"
"%PPBNG_PYTHON%" -c "import numpy; import PIL" >nul
if errorlevel 1 (
  echo ERROR: Export dependencies failed for "%PPBNG_PYTHON%". 1>&2
  echo Run .\tools\setup_export_python.cmd once. If PPBNG_PYTHON is set, correct or remove that override. 1>&2
  exit /b 2
)
"%PPBNG_PYTHON%" "%~dp0export_snapshot_images.py" %*
exit /b %errorlevel%
