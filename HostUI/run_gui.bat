@echo off
setlocal
set HERE=%~dp0
if not exist "%HERE%.venv\Scripts\python.exe" ( echo [gui] .venv not found & exit /b 1 )
"%HERE%.venv\Scripts\python.exe" "%HERE%crowerfoc_gui.py"
