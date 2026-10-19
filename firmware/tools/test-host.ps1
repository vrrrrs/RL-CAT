# RL-CAT host unit tests (PC)
#
# Compiles the HAL-independent firmware modules with the host gcc and runs the
# test binaries. This is how every phase is verified WITHOUT hardware attached.
#
# Two suites:
#   test_pure_logic  -- CRC / frame codec / phase / joint math / STS codec
#   test_servo_bus   -- servo bus transaction state machine, driven by a fake bus
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File firmware/tools/test-host.ps1
#   powershell -ExecutionPolicy Bypass -File firmware/tools/test-host.ps1 --full
#
# Exit code: 0 = all suites passed

[CmdletBinding()]
param(
    [string]$Gcc = 'D:\Software\Work\mingw64\bin\gcc.exe',
    # Extra args forwarded to every test binary (e.g. --full)
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$TestArgs = @()
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path $Gcc)) {
    $found = Get-Command gcc -ErrorAction SilentlyContinue
    if ($null -eq $found) { throw "gcc not found (looked at $Gcc and PATH)" }
    $Gcc = $found.Source
}

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projRoot  = (Resolve-Path (Join-Path $scriptDir '..\stm32')).Path
$testDir   = (Resolve-Path (Join-Path $scriptDir '..\tests\host')).Path
$outDir    = Join-Path $projRoot 'build-host'

if (Test-Path $outDir) { Remove-Item $outDir -Recurse -Force }
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

$incDir = Join-Path $projRoot 'Core\Inc'
$srcDir = Join-Path $projRoot 'Core\Src'

# Modules that must NOT include any HAL header (so they stay host-testable).
# Adding a file here is a promise: it must remain hardware-independent.
$pureSrc = @(
    'rlca_crc16.c',
    'rlca_frame.c',
    'rlca_phase.c',
    'rlca_math.c',
    'rlca_sts.c',
    'link_proto.c',
    'link_pi.c',
    'battery.c',
    'imu.c',
    'rlca_safety.c',
    'rlca_calib_proc.c'
) | ForEach-Object { Join-Path $srcDir $_ }

# servo_bus is hardware-independent too: it reaches the hardware only through
# the injected rlca_servo_port_t.
$servoSrc = Join-Path $srcDir 'servo_bus.c'

# app_mode is pure logic (it only needs link_proto for mode sanitize), but it
# asks app_safety for the estop truth, so app_safety comes along. app_safety
# reaches the world only through the pure battery_*/imu_* accessors.
$appModeSrc = @(
    'app_mode.c',
    'app_safety.c'
) | ForEach-Object { Join-Path $srcDir $_ }

# Shared host-only stub so link_pi.c can be linked without dragging in the HAL.
$stubSrc = Join-Path $testDir 'host_link_port_stub.c'

foreach ($s in ($pureSrc + $servoSrc + $appModeSrc)) {
    if (-not (Test-Path $s)) { throw "missing source: $s" }
}

$common = @(
    '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-O1', '-g',
    '-Wno-unused-parameter', "-I$incDir"
)

$suites = @(
    @{ Name = 'pure_logic'; Test = 'test_pure_logic.c'; Src = ($pureSrc + $stubSrc) },
    @{ Name = 'servo_bus';  Test = 'test_servo_bus.c';  Src = ($pureSrc + $servoSrc + $stubSrc) },
    @{ Name = 'link_proto'; Test = 'test_link_proto.c'; Src = ($pureSrc + $stubSrc) },
    @{ Name = 'link_pi';    Test = 'test_link_pi.c';    Src = ($pureSrc + $stubSrc) },
    @{ Name = 'sensors';    Test = 'test_sensors.c';    Src = ($pureSrc + $stubSrc) },
    @{ Name = 'safety_calib'; Test = 'test_safety_calib.c'; Src = ($pureSrc + $stubSrc) },
    @{ Name = 'app_mode';   Test = 'test_app_mode.c';   Src = ($pureSrc + $appModeSrc + $stubSrc) }
)

Write-Host "== RL-CAT host unit tests ==" -ForegroundColor Cyan
Write-Host ("gcc : {0}" -f $Gcc)
Write-Host ""

$prev = $ErrorActionPreference
$totalFail = 0

foreach ($s in $suites) {
    $exe = Join-Path $outDir ("test_{0}.exe" -f $s.Name)
    $argv = $common + @('-o', $exe, (Join-Path $testDir $s.Test)) + $s.Src + @('-lm')

    $ErrorActionPreference = 'Continue'
    $buildOut = & $Gcc @argv 2>&1
    $buildCode = $LASTEXITCODE
    $ErrorActionPreference = $prev

    if ($buildCode -ne 0) {
        Write-Host ("--- {0}: BUILD FAILED ---" -f $s.Name) -ForegroundColor Red
        foreach ($l in $buildOut) { Write-Host "  $l" }
        $totalFail++
        continue
    }

    $ErrorActionPreference = 'Continue'
    $runOut = & $exe @TestArgs 2>&1
    $runCode = $LASTEXITCODE
    $ErrorActionPreference = $prev

    foreach ($l in $runOut) { Write-Host $l }
    if ($runCode -ne 0) {
        Write-Host ("--- {0}: TESTS FAILED ---" -f $s.Name) -ForegroundColor Red
        $totalFail++
    }
    Write-Host ""
}

if ($totalFail -ne 0) {
    Write-Host ("{0} suite(s) FAILED" -f $totalFail) -ForegroundColor Red
    exit 1
}
Write-Host "ALL HOST TEST SUITES PASSED" -ForegroundColor Green
exit 0
