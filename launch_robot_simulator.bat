@echo off
setlocal
cd /d "%~dp0"
where python >nul 2>nul
if errorlevel 1 (
    echo Python is not available on PATH. Install Python with Tkinter enabled.
    pause
    exit /b 1
)
python -m robot_sim %*
if errorlevel 1 (
    echo The simulator returned an error.
    pause
)
