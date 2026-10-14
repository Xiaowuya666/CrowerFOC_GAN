@echo off
rem CrowerFOC 3-phase current viewer (self-contained: uses HostUI\.venv).
setlocal
set PY=%~dp0.venv\Scripts\pythonw.exe
if not exist "%PY%" set PY=%~dp0.venv\Scripts\python.exe
start "" "%PY%" "%~dp0phase_viewer.py"
endlocal
