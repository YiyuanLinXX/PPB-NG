@echo off
setlocal
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_production_acquisition.ps1" %*
exit /b %ERRORLEVEL%
