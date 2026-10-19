@echo off
rem RL-CAT ESP32 bridge bench tool -- Windows friendly entry point.
rem PowerShell execution policy often blocks .ps1 files; this wrapper calls
rem the Python tool directly, so there is no policy to fight.
rem
rem Usage (from anywhere):
rem   bench.cmd doctor
rem   bench.cmd con --host 192.168.4.1 --port 3334
rem   bench.cmd con --host 192.168.4.1 --port 3333
rem   bench.cmd verify --loopback --report D:\acc.json
rem   bench.cmd selftest
setlocal
set "REPO=%~dp0"
set "BENCH=%REPO%firmware\esp32\tools\bench.py"
set "PY=%USERPROFILE%\.platformio\penv\Scripts\python.exe"
if not exist "%BENCH%" (
  echo bench.py not found at "%BENCH%"
  exit /b 2
)
if not exist "%PY%" set "PY=python"
"%PY%" "%BENCH%" %*
exit /b %ERRORLEVEL%
