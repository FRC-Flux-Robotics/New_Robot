@echo off
rem Double-click to collect logs, or: collect-logs.cmd -Name test1 -Notes "..."
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0collect-logs.ps1" %*
pause
