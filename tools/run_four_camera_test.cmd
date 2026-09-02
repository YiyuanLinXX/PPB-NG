@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_four_camera_test.ps1" %*
exit /b %errorlevel%
