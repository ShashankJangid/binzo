@echo off
title RadioMaster Pocket Joystick Tester and Calibrator
echo Starting Joystick Diagnostic Tool...
cd /d "%~dp0"

if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" laptop_controller\joystick_tester.py %*
) else (
    python laptop_controller\joystick_tester.py %*
)

pause
