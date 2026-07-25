@echo off
setlocal EnableExtensions EnableDelayedExpansion
cd /d "%~dp0"

set "DEBUGGER_URL=http://127.0.0.1:8765"
set "NODE_EXE="
set "DEBUGGER_PID="

rem Always restart a known debugger instance. This prevents reusing a server
rem that was started in an environment where outbound UDP was restricted.
curl.exe --silent --fail --max-time 1 "%DEBUGGER_URL%/api/status" | findstr /C:"carCommandPort" >nul
if not errorlevel 1 (
  rem Finish an active recording before terminating the old process.
  curl.exe --silent --max-time 2 --request POST "%DEBUGGER_URL%/api/recording/stop" >nul 2>nul

  for /f "tokens=5" %%P in ('netstat -ano -p tcp ^| findstr /C:"127.0.0.1:8765" ^| findstr "LISTENING"') do set "DEBUGGER_PID=%%P"
  if not defined DEBUGGER_PID (
    echo [ERROR] The old debugger was found, but its process ID could not be determined.
    pause
    exit /b 1
  )

  echo Restarting Smart Car Road Debugger, old PID=!DEBUGGER_PID!...
  taskkill /PID !DEBUGGER_PID! /T /F >nul 2>nul
  if errorlevel 1 (
    echo [ERROR] The old debugger process could not be stopped.
    pause
    exit /b 1
  )
  timeout /t 1 /nobreak >nul
)

rem Do not terminate an unrelated program if it happens to own TCP 8765.
netstat -ano -p tcp | findstr /C:":8765 " | findstr "LISTENING" >nul
if not errorlevel 1 (
  echo [ERROR] TCP port 8765 is occupied by a program that is not Road Debugger.
  echo Close that program and double-click this file again.
  pause
  exit /b 1
)

rem Prefer Node.js from PATH. The fixed path is a fallback for machines where
rem Explorer does not inherit the same PATH as PowerShell.
for /f "delims=" %%I in ('where node 2^>nul') do if not defined NODE_EXE set "NODE_EXE=%%I"
if not defined NODE_EXE if exist "D:\For_codex\node.exe" set "NODE_EXE=D:\For_codex\node.exe"

if not defined NODE_EXE (
  echo [ERROR] Node.js was not found.
  echo Install Node.js 18 or newer, or place node.exe at D:\For_codex\node.exe.
  pause
  exit /b 1
)

echo Starting Smart Car Road Debugger...
echo Car command target: 192.168.43.93:8082
echo Web page: %DEBUGGER_URL%
start "Smart Car Road Debugger" /min "%NODE_EXE%" server.js

rem Wait for the new process to bind TCP 8765 before opening the browser.
for /l %%I in (1,1,8) do (
  timeout /t 1 /nobreak >nul
  curl.exe --silent --fail --max-time 1 "%DEBUGGER_URL%/api/status" | findstr /C:"carCommandPort" >nul
  if not errorlevel 1 goto debugger_ready
)

echo.
echo [ERROR] The debugger did not start within 8 seconds.
echo Check whether UDP 8080 is occupied or Node.js was blocked.
pause
exit /b 1

:debugger_ready
start "" "%DEBUGGER_URL%"
endlocal
exit /b 0
