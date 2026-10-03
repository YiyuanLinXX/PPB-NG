@echo off
setlocal
if not defined PPBNG_PYTHON set "PPBNG_PYTHON=%~dp0..\.venv-export\Scripts\python.exe"
"%PPBNG_PYTHON%" -c "import numpy; import PIL; import yaml" >nul
if errorlevel 1 (
  echo ERROR: Run tools\setup_export_python.cmd after acquisition, or set PPBNG_PYTHON to the export environment. 1>&2
  exit /b 2
)
"%PPBNG_PYTHON%" "%~dp0quicklook_cameras.py" %*
exit /b %ERRORLEVEL%
