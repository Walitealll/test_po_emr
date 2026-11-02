@echo off
setlocal
set "PYTHONUTF8=1"
set "TASK_PROTOCOL_PYTHON=%~dp0.venv_ble_adc\Scripts\python.exe"
if exist "%TASK_PROTOCOL_PYTHON%" goto dependencies
echo Creating the BLE Python environment...
set "TASK_PROTOCOL_BASE=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%TASK_PROTOCOL_BASE%" (
    "%TASK_PROTOCOL_BASE%" -m venv "%~dp0.venv_ble_adc"
    goto dependencies
)
where py >nul 2>&1
if not errorlevel 1 (
    py -3 -m venv "%~dp0.venv_ble_adc"
    goto dependencies
)
where python >nul 2>&1
if not errorlevel 1 (
    python -m venv "%~dp0.venv_ble_adc"
    goto dependencies
)
echo Python 3 is required.
goto failed

:dependencies
if not exist "%TASK_PROTOCOL_PYTHON%" goto failed
"%TASK_PROTOCOL_PYTHON%" -c "import importlib.util, sys; sys.exit(0 if all(importlib.util.find_spec(n) is not None for n in ('bleak', 'matplotlib')) else 1)"
if not errorlevel 1 goto run
"%TASK_PROTOCOL_PYTHON%" -m pip install "bleak==3.0.2" "matplotlib>=3.8,<4"
if errorlevel 1 goto failed
:run
"%TASK_PROTOCOL_PYTHON%" "%~dp0plot_ble_protocol.py" %*
if errorlevel 1 goto failed
exit /b 0
:failed
echo The application failed. See the message above.
pause
exit /b 1
