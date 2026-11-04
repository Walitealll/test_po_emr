@echo off
setlocal
cd /d "%~dp0"
set "PYTHONUTF8=1"
set "TASK_VENV=%~dp0.venv"
set "TASK_PYTHON=%TASK_VENV%\Scripts\python.exe"
if exist "%TASK_PYTHON%" goto check_libraries

echo Creating a local Python environment...
set "TASK_BUNDLED=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%TASK_BUNDLED%" (
    "%TASK_BUNDLED%" -m venv "%TASK_VENV%"
    goto environment_created
)
where py >nul 2>&1
if not errorlevel 1 (
    py -3 -m venv "%TASK_VENV%"
    goto environment_created
)
where python >nul 2>&1
if not errorlevel 1 (
    python -m venv "%TASK_VENV%"
    goto environment_created
)
echo Python 3 was not found. Install Python 3 and run this file again.
goto failed

:environment_created
if not exist "%TASK_PYTHON%" goto failed

:check_libraries
"%TASK_PYTHON%" -c "import importlib.util, sys; sys.exit(0 if all(importlib.util.find_spec(n) is not None for n in ('serial', 'bleak', 'matplotlib')) else 1)"
if not errorlevel 1 goto start_plot
echo Installing pyserial, bleak and matplotlib...
"%TASK_PYTHON%" -m pip install --disable-pip-version-check pyserial "bleak==3.0.2" "matplotlib>=3.8,<4"
if errorlevel 1 goto failed

:start_plot
"%TASK_PYTHON%" "%~dp0plot_frame.py" %*
if errorlevel 1 goto failed
exit /b 0

:failed
echo The plot could not start. See the error above.
pause
exit /b 1
