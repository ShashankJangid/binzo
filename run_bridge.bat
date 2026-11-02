@echo off
title RadioMaster Pocket to M5Core2 Mecanum Trash Can Bridge
echo Starting UDP Bridge...
cd /d "%~dp0"

if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" laptop_controller\radio_bridge.py %*
) else (
    python laptop_controller\radio_bridge.py %*
)

pause
