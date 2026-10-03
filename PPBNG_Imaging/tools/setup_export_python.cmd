@echo off
setlocal
rem Create an isolated offline export environment; never pip-install into ROS Python.
set "EXPORT_ENV=%~dp0..\.venv-export"
set "EXPORT_BASE=%~1"
if not defined EXPORT_BASE set "EXPORT_BASE=C:\pixi_ws\.pixi\envs\default\python.exe"
if not exist "%EXPORT_ENV%\Scripts\python.exe" (
  if not exist "%EXPORT_BASE%" (
    echo ERROR: Provide a stable Python 3.10+ executable: tools\setup_export_python.cmd "C:\path\python.exe" 1>&2
    exit /b 2
  )
  "%EXPORT_BASE%" -c "import sys; sys.exit(0 if sys.version_info >= (3,10) else 2)"
  if errorlevel 1 exit /b 2
  "%EXPORT_BASE%" -m venv "%EXPORT_ENV%"
  if errorlevel 1 exit /b 2
)
rem The installed conda-based bootstrap Python needs its DLL directory for pip TLS.
for %%I in ("%EXPORT_BASE%") do set "PATH=%%~dpILibrary\bin;%PATH%"
"%EXPORT_ENV%\Scripts\python.exe" -m pip install --disable-pip-version-check -r "%~dp0requirements_export.txt"
if errorlevel 1 exit /b 2
"%EXPORT_ENV%\Scripts\python.exe" -c "import sys,numpy,PIL,yaml; print('Export Python ready:',sys.executable); print('numpy',numpy.__version__,'Pillow',PIL.__version__,'PyYAML',yaml.__version__)"
if errorlevel 1 exit /b 2
echo Export commands now use .venv-export by default. No activation or ROS rebuild is required.
echo If PPBNG_PYTHON is set, it remains an explicit override.
exit /b 0
