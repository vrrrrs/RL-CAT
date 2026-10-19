# RL-CAT Python-side tests (PC)
#
# Why this exists:
#   The firmware and the Pi share ONE protocol definition. firmware/tests/host
#   asserts the byte offsets in C; this runs the Python counterpart so that a
#   drift on either side is caught immediately.
#
#   Two suites:
#     pi/selfcheck.py     -- protocol codec + 47-dim observation assembly
#     pi/test_loopback.py -- the serial simulator driven by a fake STM32 over a
#                            socket pair (no hardware, no pyserial needed)
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File firmware/tools/test-python.ps1
#
# Exit code: 0 = all suites passed

[CmdletBinding()]
param(
    [string]$Python = 'python'
)

$ErrorActionPreference = 'Stop'

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot  = (Resolve-Path (Join-Path $scriptDir '..\..')).Path
$piDir     = Join-Path $repoRoot 'pi'

foreach ($f in @('rlcat_proto.py', 'selfcheck.py', 'test_loopback.py')) {
    $p = Join-Path $piDir $f
    if (-not (Test-Path $p)) { throw "missing $p" }
}

# Version check: the simulator needs >= 3.7 for dataclasses; we use 3.10+ syntax.
$prev = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
$ver = & $Python -c "import sys; print('%d.%d' % sys.version_info[:2])" 2>&1
$ErrorActionPreference = $prev

if ($LASTEXITCODE -ne 0) { throw "python not runnable: $ver" }
Write-Host "== RL-CAT Python tests ==" -ForegroundColor Cyan
Write-Host ("python : {0} ({1})" -f $Python, $ver)
Write-Host ""

$suites = @(
    @{ Name = 'selfcheck'; Path = (Join-Path $piDir 'selfcheck.py') },
    @{ Name = 'loopback';  Path = (Join-Path $piDir 'test_loopback.py') }
)

$fail = 0
foreach ($s in $suites) {
    Write-Host ("--- {0} ---" -f $s.Name)
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    $out = & $Python $s.Path 2>&1
    $code = $LASTEXITCODE
    $ErrorActionPreference = $prev

    foreach ($l in $out) { Write-Host $l }
    if ($code -ne 0) {
        Write-Host ("--- {0}: FAILED (exit {1}) ---" -f $s.Name, $code) -ForegroundColor Red
        $fail++
    }
    Write-Host ""
}

if ($fail -ne 0) {
    Write-Host ("{0} suite(s) FAILED" -f $fail) -ForegroundColor Red
    exit 1
}
Write-Host "ALL PYTHON TEST SUITES PASSED" -ForegroundColor Green
exit 0
