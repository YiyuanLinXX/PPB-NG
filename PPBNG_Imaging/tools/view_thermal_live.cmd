@echo off
setlocal
if not defined PPBNG_PYTHON set "PPBNG_PYTHON=%~dp0..\.venv-export\Scripts\python.exe"
"%PPBNG_PYTHON%" "%~dp0view_thermal_live.py" %*
exit /b %ERRORLEVEL%
