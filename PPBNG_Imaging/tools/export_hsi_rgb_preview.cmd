@echo off
setlocal
rem Use a stable workspace environment, not a replaceable application cache.
if not defined PPBNG_PYTHON set "PPBNG_PYTHON=%~dp0..\.venv-export\Scripts\python.exe"
"%PPBNG_PYTHON%" -c "import numpy; import PIL; import yaml" >nul
if errorlevel 1 (
  echo ERROR: Export dependencies failed for "%PPBNG_PYTHON%". 1>&2
  echo Run .\tools\setup_export_python.cmd once. If PPBNG_PYTHON is set, correct or remove that override. 1>&2
  exit /b 2
)
"%PPBNG_PYTHON%" "%~dp0export_hsi_rgb_preview.py" %*
exit /b %errorlevel%
