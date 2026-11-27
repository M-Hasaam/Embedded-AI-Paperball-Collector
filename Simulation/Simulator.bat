@echo off
rem Opens the simulator window. Needs Python 3.12 (the py launcher) with Pillow installed.
cd /d "%~dp0"
py -3.12 -m sim.gui
if errorlevel 1 pause
