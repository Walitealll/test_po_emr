@echo off
setlocal
cd /d "%~dp0"
set "PYTHONUTF8=1"
where py >nul 2>&1
if errorlevel 1 goto no_python
py -3 plot_frame.py %*
if errorlevel 1 goto failed
exit /b 0

:no_python
echo Python 3 was not found.
goto failed

:failed
echo Frame plot did not start. See the error above.
pause
exit /b 1
