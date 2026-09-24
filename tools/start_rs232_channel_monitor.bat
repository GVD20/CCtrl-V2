@echo off
cd /d "%~dp0\.."
python tools\rs232_channel_monitor.py
if errorlevel 1 pause
