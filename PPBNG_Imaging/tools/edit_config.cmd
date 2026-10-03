@echo off
rem Open the single authoritative operator configuration. Never opens hardware.
start "" notepad.exe "%~dp0..\src\ppbng_bringup\config\ppbng_config.yaml"
