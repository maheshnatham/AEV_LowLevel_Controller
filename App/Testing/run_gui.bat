@echo off
title AEV Controller Diagnostic GUI
cd /d "%~dp0"
echo Starting AEV Diagnostic GUI & Protocol Inspector...
python aev_diagnostic_gui.py
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo Application exited with an error.
    pause
)
