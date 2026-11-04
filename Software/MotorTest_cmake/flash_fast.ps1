param(
    [string]$FirmwarePath,
    [string]$ProbeUid,
    [switch]$SkipBuild,
    [switch]$ListOnly,
    [switch]$SkipVerify
)

$ErrorActionPreference = 'Stop'
$Target = 'stm32c562cet6'
$BuildPreset = 'debug_GCC_STM32C562CET6'
$BuildDir = Join-Path $PSScriptRoot "build\$BuildPreset"
$DefaultElf = Join-Path $BuildDir 'MotorTest.elf'
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$PyOcdPython = Join-Path $RepoRoot '.tools\pyocd-venv\Scripts\python.exe'
$HostPython = Join-Path $RepoRoot 'HostUI\.venv\Scripts\python.exe'

function Stop-Flash([string]$Message, [int]$Code = 1) {
    Write-Host "[flash] ERROR: $Message" -ForegroundColor Red
    exit $Code
}

if (-not (Test-Path -LiteralPath $PyOcdPython)) {
    $PythonCommand = Get-Command python.exe -ErrorAction SilentlyContinue
    if ($null -eq $PythonCommand) { Stop-Flash "找不到 Python：$PyOcdPython" 2 }
    $PyOcdPython = $PythonCommand.Source
}

function Invoke-PyOcd([string[]]$Arguments) {
    # Windows PowerShell 5 turns native stderr lines into ErrorRecords. Keep
    # them as captured output: pyOCD emits harmless warnings on stderr while
    # successfully falling back to an FLM flash algorithm.
    $PreviousErrorActionPreference = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $Output = & $PyOcdPython -m pyocd @Arguments 2>&1
        $Code = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $PreviousErrorActionPreference
    }
    $Text = ($Output | ForEach-Object { "$_" }) -join "`n"
    return [pscustomobject]@{ ExitCode = $Code; Text = $Text }
}

function Find-HorcoUid([string]$Text) {
    foreach ($Line in ($Text -split "`r?`n")) {
        if ($Line -match '(?i)Horco') {
            $Match = [regex]::Match($Line, '(?<!\d)(\d{10,})(?!\d)')
            if ($Match.Success) { return $Match.Groups[1].Value }
        }
    }
    return $null
}

Write-Host '[flash] 正在识别 Horco CMSIS-DAP 探针…'
$ProbeResult = Invoke-PyOcd @('list', '-p', '--no-config')
if ($ProbeResult.Text) { Write-Host $ProbeResult.Text }
if ($ProbeResult.ExitCode -ne 0) {
    Stop-Flash 'pyOCD 无法枚举调试探针。请关闭占用探针的调试软件后重试。' 3
}

$DetectedUid = $null
if ($ProbeUid) {
    if ($ProbeResult.Text -notmatch [regex]::Escape($ProbeUid)) {
        Stop-Flash "指定探针 UID $ProbeUid 当前未连接。" 3
    }
    $DetectedUid = $ProbeUid
} else {
    $DetectedUid = Find-HorcoUid $ProbeResult.Text
    if (-not $DetectedUid) {
        Stop-Flash '没有识别到 Horco CMSIS-DAP。请确认设备管理器中探针在线，或用 -ProbeUid 指定探针。' 3
    }
}
Write-Host "[flash] 已识别 Horco 探针 UID=$DetectedUid"
if ($ListOnly) { exit 0 }

if (-not $SkipBuild) {
    Write-Host "[flash] 构建固件 ($BuildPreset)…"
    Push-Location $PSScriptRoot
    try {
        if (-not (Test-Path -LiteralPath (Join-Path $BuildDir 'CMakeCache.txt'))) {
            & cmake --preset $BuildPreset
            if ($LASTEXITCODE -ne 0) { Stop-Flash 'CMake 配置失败。' 4 }
        }
        & cmake --build --preset $BuildPreset --parallel
        if ($LASTEXITCODE -ne 0) { Stop-Flash '固件构建失败，未进行烧录。' 4 }
    } finally {
        Pop-Location
    }
}

if ($FirmwarePath) {
    if (-not (Test-Path -LiteralPath $FirmwarePath -PathType Leaf)) {
        Stop-Flash "固件文件不存在：$FirmwarePath" 5
    }
    $Elf = (Resolve-Path -LiteralPath $FirmwarePath).Path
} else {
    $Elf = $DefaultElf
}
if (-not (Test-Path -LiteralPath $Elf -PathType Leaf)) {
    Stop-Flash "找不到固件 ELF：$Elf" 5
}
Write-Host "[flash] 固件：$Elf"

$Frequencies = @(1000000, 500000, 250000, 100000)
$Flashed = $false
$UsbFailure = $false
foreach ($Frequency in $Frequencies) {
    # A new pyOCD process is used for every attempt so a failed USB transaction
    # cannot leave a stale CMSIS-DAP handle in the next retry.
    for ($Attempt = 1; $Attempt -le 2; $Attempt++) {
        Write-Host "[flash] 烧录尝试：SWD $Frequency Hz，第 $Attempt 次"
        $FlashResult = Invoke-PyOcd @(
            'load', '--no-config', '--target', $Target, '--uid', $DetectedUid,
            '--frequency', "$Frequency", '--connect', 'under-reset',
            '--no-reset', $Elf
        )
        if ($FlashResult.Text) { Write-Host $FlashResult.Text }
        if ($FlashResult.ExitCode -eq 0) {
            $Flashed = $true
            break
        }

        if ($FlashResult.Text -match '(?i)Timeout reading from probe|AssertionError|No available debug probes|could not open device') {
            if ($Attempt -lt 2) {
                Write-Host '[flash] 探针 USB 访问短暂失败，重新枚举后重试…' -ForegroundColor Yellow
                Start-Sleep -Milliseconds 900
                $Recheck = Invoke-PyOcd @('list', '-p', '--no-config')
                if ($Recheck.ExitCode -ne 0 -or $Recheck.Text -notmatch [regex]::Escape($DetectedUid)) {
                    $UsbFailure = $true
                    break
                }
                continue
            }
            $UsbFailure = $true
            break
        }

        if ($Attempt -lt 2) {
            Write-Host '[flash] 本次连接失败，重新打开探针后重试…' -ForegroundColor Yellow
            Start-Sleep -Milliseconds 700
        }
    }
    if ($Flashed -or $UsbFailure) { break }
}

if (-not $Flashed) {
    if ($UsbFailure) {
        Stop-Flash '探针枚举正常但 USB 命令超时。脚本已自动重试；请检查 Horco 与目标板供电/连接，再运行一次。' 6
    }
    Stop-Flash '所有 SWD 频率均烧录失败。' 6
}
Write-Host '[flash] 烧录成功。'

Write-Host '[flash] 复位目标…'
$ResetResult = Invoke-PyOcd @('reset', '--no-config', '--target', $Target, '--uid', $DetectedUid, '--method', 'sysresetreq', '--frequency', '100000')
if ($ResetResult.Text) { Write-Host $ResetResult.Text }
if ($ResetResult.ExitCode -ne 0) {
    Write-Host '[flash] 系统复位失败，尝试硬件复位…' -ForegroundColor Yellow
    $ResetResult = Invoke-PyOcd @('reset', '--no-config', '--target', $Target, '--uid', $DetectedUid, '--method', 'hw', '--frequency', '100000')
    if ($ResetResult.Text) { Write-Host $ResetResult.Text }
    if ($ResetResult.ExitCode -ne 0) {
        Stop-Flash '固件已写入，但两种复位方式都失败；请手动按板上复位键。' 7
    }
}
Write-Host '[flash] 复位成功。'

if (-not $SkipVerify -and (Test-Path -LiteralPath $HostPython)) {
    Write-Host '[flash] 被动检查 Horco 无线串口遥测（921600，约 0.8 秒）…'
    $VerifyCode = @'
import time, sys
try:
    import serial
    from serial.tools import list_ports
    ports = [p for p in list_ports.comports() if p.vid == 0xFAED and p.pid == 0x4870]
    if not ports:
        print("UART_VERIFY_NO_HORCO_COM")
        sys.exit(0)
    port = ports[0]
    with serial.Serial(port.device, 921600, timeout=0.05) as s:
        data = bytearray()
        end = time.monotonic() + 0.8
        while time.monotonic() < end:
            data.extend(s.read(4096))
    print(f"UART_VERIFY {port.device} bytes={len(data)}")
except Exception as exc:
    print("UART_VERIFY_SKIPPED", type(exc).__name__, str(exc))
'@
    # Pass the embedded Python source as base64. Windows PowerShell's native
    # argument quoting otherwise strips quotes inside a multiline -c string.
    $VerifyCodeBase64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($VerifyCode))
    $VerifyLauncher = "exec(__import__('base64').b64decode('$VerifyCodeBase64').decode('utf-8'))"
    $PreviousErrorActionPreference = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $VerifyOutput = & $HostPython -c $VerifyLauncher 2>&1
        $VerifyCodeExit = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $PreviousErrorActionPreference
    }
    if ($VerifyOutput) { $VerifyOutput | ForEach-Object { Write-Host "[flash] $_" } }
    if ($VerifyCodeExit -ne 0 -or ($VerifyOutput -match 'UART_VERIFY_SKIPPED|UART_VERIFY_NO_HORCO_COM')) {
        Write-Host '[flash] 无线串口验证未完成；烧录与复位已成功。' -ForegroundColor Yellow
    } elseif ($VerifyOutput -match 'UART_VERIFY .*bytes=([1-9]\d*)') {
        Write-Host '[flash] UART 遥测验证成功。' -ForegroundColor Green
    } else {
        Write-Host '[flash] COM 端口可访问但没有遥测；请核对目标串口接线和固件状态。' -ForegroundColor Yellow
    }
}

Write-Host '[flash] 全部完成。没有发送任何电机控制命令。' -ForegroundColor Green
