@echo off
cd /d "%~dp0"
call "%~dp0start_frame.cmd" --ble %*
