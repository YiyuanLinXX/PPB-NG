@echo off
setlocal
rem Build the validated production graph from an empty cache with real SDKs.
rem Compilation and unit tests do not open or start any hardware.
call "%~dp0build.cmd" --cmake-args -DPython3_EXECUTABLE=C:/pixi_ws/.pixi/envs/default/python.exe -DPPBNG_HSI_ENABLE_SPECSENSOR=ON -DPPBNG_RGB_ENABLE_SPINNAKER=ON -DPPBNG_THERMAL_ENABLE_SPINNAKER=ON -DBUILD_TESTING=ON %*
exit /b %ERRORLEVEL%
