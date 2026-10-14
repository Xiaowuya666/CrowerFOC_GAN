param([string]$port = 'COM7')

$sp = New-Object System.IO.Ports.SerialPort $port, 921600, 'None', 8, 'One'
$sp.ReadTimeout = 300
$sp.WriteTimeout = 1000
$sp.ReadBufferSize = 65536
$sp.Open()

# ---------- Phase 1: TX throughput ----------
$buf = New-Object byte[] 65536
$sw = [Diagnostics.Stopwatch]::StartNew()
[long]$total = 0
while ($sw.Elapsed.TotalSeconds -lt 5.0) {
  try { $total += $sp.Read($buf, 0, $buf.Length) } catch {}
}
$sw.Stop()
$secs = $sw.Elapsed.TotalSeconds
Write-Output ("TX: {0:N0} bytes in {1:N2} s = {2:N0} bytes/s = {3:N2} Mbit/s" -f $total, $secs, ($total / $secs), ($total * 8.0 / $secs / 1e6))

# ---------- Phase 2: RX + echo ----------
$msg = [Text.Encoding]::ASCII.GetBytes('HELLO1234')
$sp.Write($msg, 0, $msg.Length)

$acc = New-Object 'System.Collections.Generic.List[byte]'
$sw2 = [Diagnostics.Stopwatch]::StartNew()
while ($sw2.Elapsed.TotalSeconds -lt 1.5) {
  try {
    $n = $sp.Read($buf, 0, $buf.Length)
    for ($i = 0; $i -lt $n; $i++) { $acc.Add($buf[$i]) }
    if ($acc.Count -gt 300000) { break }
  } catch {}
}
$sp.Close()

$arr = $acc.ToArray()
$txt = [Text.Encoding]::ASCII.GetString($arr)
$idx = $txt.IndexOf('ECHO')
if ($idx -ge 0) {
  $end = [Math]::Min($idx + 4 + 15, $arr.Length - 1)
  Write-Output ("RX: ECHO marker found; bytes after marker: " + ([BitConverter]::ToString($arr[($idx+4)..$end])))
  Write-Output ("RX: decoded = '" + [Text.Encoding]::ASCII.GetString($arr[($idx+4)..$end]) + "'")
} else {
  Write-Output ("RX: no ECHO marker (received {0:N0} bytes)" -f $arr.Length)
}
