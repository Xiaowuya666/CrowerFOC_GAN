@echo off
setlocal
rem Flash MotorTest.elf to STM32C562CET6 via DAPLink (CMSIS-DAP), then
rem perform an explicit software reset (SYSRESETREQ) and verify the core runs.
rem
rem --no-reset on 'flash' avoids relying on the probe's default reset.
rem 'reset --method sysresetreq' uses the ARM core SYSRESETREQ, which does not
rem depend on the nRESET pin being wired to the board.

set PYOCD=C:\Users\Crowe\Desktop\MyMotor_ws\.tools\pyocd-venv\Scripts\pyocd.exe
set ELF=%~dp0build\debug_GCC_STM32C562CET6\MotorTest.elf
set TARGET=stm32c562cet6

if not exist "%PYOCD%" ( echo [flash] pyocd not found: %PYOCD% & exit /b 1 )
if not exist "%ELF%"   ( echo [flash] elf not found: %ELF% & echo build first: cmake --build --preset debug_GCC_STM32C562CET6 & exit /b 1 )

echo [flash] programming %ELF%
"%PYOCD%" flash --target %TARGET% --no-reset "%ELF%"
if errorlevel 1 ( echo [flash] FAILED & exit /b 1 )

echo [flash] reset (sysresetreq)
"%PYOCD%" reset --target %TARGET% --method sysresetreq
if errorlevel 1 ( echo [flash] reset FAILED & exit /b 1 )

echo [flash] core state:
"%PYOCD%" commander -M attach -t %TARGET% -c "status"

echo [flash] done
