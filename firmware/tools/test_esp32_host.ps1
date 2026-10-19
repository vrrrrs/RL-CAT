# Host unit tests for the ESP32 firmware's pure-logic layers.
#
# Why: the frame/CRC layer and the injection gate are pure logic, and the gate
# decides whether the ESP32 may command the robot.  Running them on the PC
# (MinGW gcc) makes the P2/P3 acceptance criteria verifiable without the board
# or the STM32, and cross-checks the firmware encoder against the Python
# reference from the protocol spec.
#
# Usage (from anywhere):
#   powershell -ExecutionPolicy Bypass -File firmware/tools/test_esp32_host.ps1
#
# Exit code: 0 = all green; 1 = test failure; 2 = build failure / toolchain missing

[CmdletBinding()]
param(
    [string]$Gcc = 'D:\Software\Work\mingw64\bin\gcc.exe',
    [string]$Python = 'C:\Python314\python.exe'
)

$ErrorActionPreference = 'Stop'

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot  = (Resolve-Path (Join-Path $scriptDir '..\..')).Path
$projDir   = Join-Path $repoRoot 'firmware\esp32'
$buildDir  = Join-Path $projDir 'build-host'

foreach ($t in @($Gcc, $Python)) {
    if (-not (Test-Path $t)) { Write-Host "toolchain missing: $t" -ForegroundColor Red; exit 2 }
}

New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

$targets = @(
    @{ name = 'test_protocol';     srcs = @('src\protocol.c', 'test\test_protocol.c'); crosscheck = $true  },
    @{ name = 'test_arbiter_gate'; srcs = @('test\test_arbiter_gate.c');               crosscheck = $false },
    @{ name = 'test_cfg';          srcs = @('test\test_cfg.c');                        crosscheck = $false }
)

$failures = 0
$outputs  = @{}

foreach ($t in $targets) {
    $exe = Join-Path $buildDir ($t.name + '.exe')
    Write-Host ("== building {0} ==" -f $t.name) -ForegroundColor Cyan

    $gccArgs = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-Os', '-g',
                 '-I', (Join-Path $projDir 'include'))
    foreach ($s in $t.srcs) { $gccArgs += (Join-Path $projDir $s) }
    $gccArgs += @('-o', $exe, '-lm')

    $build = & $Gcc @gccArgs 2>&1
    if ($LASTEXITCODE -ne 0) {
        $build | ForEach-Object { Write-Host "  $_" -ForegroundColor Red }
        Write-Host ("BUILD FAILED: {0}" -f $t.name) -ForegroundColor Red
        exit 2
    }

    $out = & $exe 2>&1
    $code = $LASTEXITCODE
    $out | ForEach-Object { Write-Host "  $_" }
    $outputs[$t.name] = $out
    if ($code -ne 0) { $failures++ }
}

# ---- independent oracle: firmware encoder vs the protocol spec's Python -----
Write-Host '== cross-check frames against the Python reference ==' -ForegroundColor Cyan
$cFrames = @($outputs['test_protocol'] | Where-Object { $_ -like 'FRAME *' } | ForEach-Object { $_.Trim() })
$pyFrames = @(& $Python (Join-Path $projDir 'test\reference_frames.py') 2>&1 |
              Where-Object { $_ -like 'FRAME *' } | ForEach-Object { $_.Trim() })

$crossOk = $true
if ($cFrames.Count -ne $pyFrames.Count) {
    Write-Host ("  FAIL frame count: firmware {0} vs python {1}" -f $cFrames.Count, $pyFrames.Count) -ForegroundColor Red
    $crossOk = $false
} else {
    for ($i = 0; $i -lt $cFrames.Count; $i++) {
        if ($cFrames[$i] -ne $pyFrames[$i]) {
            Write-Host "  FAIL $($cFrames[$i].Split(' ')[1])" -ForegroundColor Red
            Write-Host "    firmware: $($cFrames[$i])" -ForegroundColor Red
            Write-Host "    python  : $($pyFrames[$i])" -ForegroundColor Red
            $crossOk = $false
        } else {
            $hex = $cFrames[$i].Split(' ')[2]
            $preview = if ($hex.Length -gt 24) { $hex.Substring(0, 24) + '...' } else { $hex }
            Write-Host "  ok   $($cFrames[$i].Split(' ')[1])  $preview" -ForegroundColor DarkGray
        }
    }
}

if ($failures -eq 0 -and $crossOk) {
    Write-Host ''
    Write-Host 'HOST TESTS OK (unit tests + python cross-check)' -ForegroundColor Green
    exit 0
}

Write-Host ''
Write-Host ("HOST TESTS FAILED ({0} failing target(s), crosscheck ok={1})" -f $failures, $crossOk) -ForegroundColor Red
exit 1
