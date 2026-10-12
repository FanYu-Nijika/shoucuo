@echo off
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0record_serial.ps1" -Port COM32 -BaudRate 115200
echo.
pause
