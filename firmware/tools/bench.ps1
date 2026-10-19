# Run the PC-side ESP32 bench tool with a Python that has pyserial.
#
# Why a wrapper: the system Python (C:\Python314) does not have pyserial, while
# PlatformIO's venv Python does.  This script picks the first interpreter that
# can import pyserial and forwards every argument to
# firmware/esp32/tools/bench.py.
#
# Usage (from anywhere):
#   powershell -ExecutionPolicy Bypass -File firmware/tools/bench.ps1 selftest
#   powershell -ExecutionPolicy Bypass -File firmware/tools/bench.ps1 mon --port COM9
#   powershell -ExecutionPolicy Bypass -File firmware/tools/bench.ps1 sim --port COM9
#   powershell -ExecutionPolicy Bypass -File firmware/tools/bench.ps1 loop --host 192.168.4.1
#   powershell -ExecutionPolicy Bypass -File firmware/tools/bench.ps1 con --cmd "?" --cmd stats

[CmdletBinding()]
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$BenchArgs
)

$ErrorActionPreference = 'Stop'

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot  = (Resolve-Path (Join-Path $scriptDir '..\..')).Path
$bench     = Join-Path $repoRoot 'firmware\esp32\tools\bench.py'

if (-not (Test-Path $bench)) { throw "bench.py not found at $bench" }

$candidates = @(
    (Join-Path $env:USERPROFILE '.platformio\penv\Scripts\python.exe'),
    'C:\Python314\python.exe',
    'python'
)

$python = $null
foreach ($c in $candidates) {
    try {
        $null = & $c -c 'import serial' 2>$null
        if ($LASTEXITCODE -eq 0) { $python = $c; break }
    } catch { }
}
if (-not $python) {
    Write-Host 'No Python with pyserial found (tried PlatformIO venv and C:\Python314).' -ForegroundColor Red
    Write-Host 'Install it with:  <python> -m pip install pyserial' -ForegroundColor Yellow
    exit 2
}

& $python $bench @BenchArgs
exit $LASTEXITCODE
