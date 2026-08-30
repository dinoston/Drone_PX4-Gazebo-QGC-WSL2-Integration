@echo off
cd /d "%~dp0"
"%LOCALAPPDATA%\Programs\Python\Python311\python.exe" ui\mission_control.py
if errorlevel 1 (
  echo.
  echo Mission Control exited with an error.
  pause
)
