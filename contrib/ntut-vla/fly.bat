@echo off
rem Double-click launcher: AirSim (urban) + VLA drone with guardrail ON.
rem For options run fly.ps1 from PowerShell instead.
powershell -ExecutionPolicy Bypass -File "%~dp0fly.ps1" %*
pause
