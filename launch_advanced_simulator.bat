@echo off
setlocal
cd /d "%~dp0"
where python >nul 2>nul
if errorlevel 1 (
    echo Python not found in PATH. Install Python with Tkinter support.
    pause
    exit /b 1
)
python -m robot_sim --advanced %*
if errorlevel 1 (
    echo Advanced 3D simulator failed. Please record the error for verification.
    pause
)
