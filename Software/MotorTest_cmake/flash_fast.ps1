# Fast, reliable flash + run for MotorTest (STM32C562CET6 via CMSIS-DAP).
# Uses "python -m pyocd" because the pyocd.exe launcher is broken on this box.
# Codex/Windows note: the sandbox may hide WinUSB/libusb devices even while
# Device Manager lists Horco CMSIS-DAP v2 (VID/PID FAED:4870). Run this script
# with direct USB access. Do not treat a sandbox-only "no probes" result as a
# reason to replug or reinstall drivers; enumerate with `pyocd list -p` using
# the same direct USB access first.
# - tries fast SWD clocks first, falls back if the link is marginal
# - hardware reset with retries so USB re-enumerates after programming
$ErrorActionPreference = 'Continue'
$py  = 'C:\Users\Crowe\Desktop\MyMotor_ws\.tools\pyocd-venv\Scripts\python.exe'
$elf = Join-Path $PSScriptRoot 'build\debug_GCC_STM32C562CET6\MotorTest.elf'
$tgt = 'stm32c562cet6'

if (-not (Test-Path $elf)) { Write-Error "elf not found: $elf"; exit 1 }

function PYOCD { & $py -m pyocd @args }

# Fail fast when USB access is unavailable. Without this check, the frequency
# fallback loop only repeats the same probe-enumeration failure several times.
$probeOutput = PYOCD list -p 2>&1
$probeExit = $LASTEXITCODE
if ($probeExit -ne 0 -or ($probeOutput -match 'No available debug probes')) {
    Write-Error '[flash] CMSIS-DAP is not visible to this process. In Codex, rerun with direct USB access; a sandbox-only miss is not evidence that the probe is disconnected.'
    $probeOutput | ForEach-Object { Write-Host $_ }
    exit 2
}
Write-Host '[flash] CMSIS-DAP detected'

$freqs = @(1000000, 500000, 250000, 100000)
$flashed = $false
foreach ($f in $freqs) {
    $t0 = Get-Date
    # Connect while reset is asserted. 1MHz completed in ~8s; faster attempts
    # on this wireless probe intermittently timed out during erase/program.
    $flashOutput = PYOCD flash --target $tgt -M under-reset -f $f $elf 2>&1
    $flashExit = $LASTEXITCODE
    $el = [math]::Round(((Get-Date) - $t0).TotalSeconds, 1)
    if ($flashExit -eq 0) { Write-Host "[flash] OK @ ${f}Hz  (${el}s)"; $flashed = $true; break }
    Write-Host "[flash] fail @ ${f}Hz  (${el}s)"
    $flashOutput | Select-Object -Last 6 | ForEach-Object { Write-Host $_ }
}
if (-not $flashed) { Write-Error '[flash] all frequencies failed'; exit 1 }

$done = $false
for ($i = 1; $i -le 8; $i++) {
    PYOCD reset --target $tgt --method hw -f 100000 *> $null
    if ($LASTEXITCODE -eq 0) { Write-Host "[flash] reset OK (try $i)"; $done = $true; break }
    Start-Sleep -Milliseconds 400
}
if (-not $done) { Write-Host '[flash] reset command failed (board may be halted)' }

# Verify the application's USB enumeration without another debug attachment.
# On this wireless probe a fresh attachment disturbed the running USB state.
for ($i = 1; $i -le 15; $i++) {
    $o = & "$PSScriptRoot\..\..\HostUI\.venv\Scripts\python.exe" -c "import serial.tools.list_ports; print(','.join(p.device for p in serial.tools.list_ports.comports() if p.vid==0x0483 and p.pid==0x5710))" 2>&1
    if ($o -match 'COM\d+') { Write-Host "[flash] USB enumerated: $o"; break }
    Start-Sleep -Milliseconds 200
}
if ($o -notmatch 'COM\d+') { Write-Warning '[flash] USB not enumerated; application running state not confirmed' }
Write-Host '[flash] done'
