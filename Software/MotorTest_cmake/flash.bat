@echo off
setlocal EnableExtensions
title CrowerFOC - Identify and Flash
set "SCRIPT_DIR=%~dp0"

set "PS_ARGS="
if "%~1"=="" goto run
if /I "%~1"=="help" goto help
if /I "%~1"=="/?" goto help
if /I "%~1"=="-h" goto help

:parse
if "%~1"=="" goto run
if /I "%~1"=="list" (
    set "PS_ARGS=%PS_ARGS% -ListOnly"
    shift
    goto parse
)
if /I "%~1"=="--list" (
    set "PS_ARGS=%PS_ARGS% -ListOnly"
    shift
    goto parse
)
if /I "%~1"=="skip-build" (
    set "PS_ARGS=%PS_ARGS% -SkipBuild"
    shift
    goto parse
)
if /I "%~1"=="--skip-build" (
    set "PS_ARGS=%PS_ARGS% -SkipBuild"
    shift
    goto parse
)
if /I "%~1"=="skip-verify" (
    set "PS_ARGS=%PS_ARGS% -SkipVerify"
    shift
    goto parse
)
if /I "%~1"=="--skip-verify" (
    set "PS_ARGS=%PS_ARGS% -SkipVerify"
    shift
    goto parse
)
if /I "%~1"=="uid" goto parse_uid
if /I "%~1"=="--uid" goto parse_uid
if /I "%~1"=="firmware" goto parse_firmware
if /I "%~1"=="--firmware" goto parse_firmware
echo [flash] Unknown option: %~1
goto help_error

:parse_uid
if "%~2"=="" echo [flash] Missing UID value.& goto help_error
set "PS_ARGS=%PS_ARGS% -ProbeUid "%~2""
shift
shift
goto parse

:parse_firmware
if "%~2"=="" echo [flash] Missing ELF path.& goto help_error
set "PS_ARGS=%PS_ARGS% -FirmwarePath "%~2""
shift
shift
goto parse

:run
echo [flash] Running from %SCRIPT_DIR%
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%flash_fast.ps1" %PS_ARGS%
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" (
    echo.
    echo [flash] Did not complete. Read the error above.
)
echo.
pause
exit /b %RESULT%

:help
echo Usage from Command Prompt:
echo   flash.bat                         Build, detect probe, flash, reset, verify UART
echo   flash.bat list                    Detect probe only; no build or flash
echo   flash.bat uid 846703532218        Select a specific probe UID
echo   flash.bat skip-build              Flash the existing default ELF
echo   flash.bat firmware "C:\path\MotorTest.elf"
echo   flash.bat skip-verify             Skip passive UART verification
echo Options may be combined, for example: flash.bat uid 846703532218 skip-build
echo.
echo This BAT file is the user entry point. It runs the PowerShell implementation internally.
exit /b 0

:help_error
echo.
call "%~f0" help
exit /b 2
