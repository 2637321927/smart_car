@echo off
setlocal
cd /d "%~dp0"

where node >nul 2>nul
if errorlevel 1 (
  echo [ERROR] Node.js not found. Please install Node.js 18 or newer.
  pause
  exit /b 1
)

echo Starting Smart Car Road Debugger...
start "" cmd /c "timeout /t 1 /nobreak >nul && start http://127.0.0.1:8765"
node server.js

if errorlevel 1 pause
endlocal
