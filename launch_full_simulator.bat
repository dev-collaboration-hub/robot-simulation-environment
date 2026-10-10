@echo off
cd /d "%~dp0"
python -m robot_sim --full
if errorlevel 1 pause
