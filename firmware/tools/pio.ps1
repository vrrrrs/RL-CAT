# run PlatformIO with the RL-CAT directory layout
#
# Why this wrapper exists:
#   1. PlatformIO's default core directory is %USERPROFILE%\.platformio, which is
#      not writable from a sandboxed session (HomeDirPermissionsError).
#   2. The DSH file sandbox labels the session workspace with a *Low* mandatory
#      integrity level.  Any executable that lives inside the workspace inherits
#      Low IL at run time, and a Low-IL process cannot write to Medium-IL paths
#      (Windows "no write up"), which makes ESP-IDF's python steps fail with
#      "PermissionError: [Errno 13]" as soon as the build directory is off the
#      workspace.  Keeping the PlatformIO core dir (and therefore the ESP-IDF
#      virtualenv python) on D: keeps every build tool at Medium IL.
#
# Layout:
#   PLATFORMIO_CORE_DIR      = D:\Work\EspIDF\core         (state, caches, IDF venv)
#   PLATFORMIO_PACKAGES_DIR  = D:\Work\EspIDF\packages     (framework + toolchains)
#   PLATFORMIO_PLATFORMS_DIR = D:\Work\EspIDF\platforms
#   build_dir                = D:\Work\EspIDF\build        (set in platformio.ini;
#                             must be whitespace-free for ESP-IDF)
#
# Usage (from firmware/esp32):
#   powershell -ExecutionPolicy Bypass -File ../tools/pio.ps1 run
#   powershell -ExecutionPolicy Bypass -File ../tools/pio.ps1 run -t upload
#   powershell -ExecutionPolicy Bypass -File ../tools/pio.ps1 device monitor

[CmdletBinding()]
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$PioArgs
)

$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$pioExe   = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\pio.exe'

if (-not (Test-Path $pioExe)) { throw "PlatformIO not found at $pioExe" }

$env:PLATFORMIO_CORE_DIR      = 'D:\Work\EspIDF\core'
$env:PLATFORMIO_PACKAGES_DIR  = 'D:\Work\EspIDF\packages'
$env:PLATFORMIO_PLATFORMS_DIR = 'D:\Work\EspIDF\platforms'

New-Item -ItemType Directory -Force -Path $env:PLATFORMIO_CORE_DIR      | Out-Null
New-Item -ItemType Directory -Force -Path $env:PLATFORMIO_PACKAGES_DIR  | Out-Null
New-Item -ItemType Directory -Force -Path $env:PLATFORMIO_PLATFORMS_DIR | Out-Null

# ---- work around ESP-IDF's "no whitespace in paths" rule ------------------
# IDF rejects space-containing project paths (and its generated
# -fmacro-prefix-map=<project dir> argument breaks).  This repo lives under
# "C:\Users\<user name>\...", so when the path contains a space we expose the repo
# through a SUBST drive letter and build from there.  SUBST is used instead of a
# directory junction on purpose: the alias survives NTFS path resolution long
# enough for CMake/Ninja, which a junction would not (junctions resolve to the
# real target, spaces and all).  The drive mapping lives as long as the Windows
# logon session; this script re-creates it when missing.
$projRel = Join-Path 'firmware' 'esp32'
$projDir = Join-Path $repoRoot $projRel

if ($repoRoot -match '\s') {
    $aliasDrive = $null
    foreach ($letter in @('R', 'Q', 'P', 'O', 'N', 'M')) {
        $candidate = "${letter}:"
        $existing = (& subst) 2>$null | Where-Object { $_ -like "$candidate*" }
        if ($existing) {
            if ($existing -match [regex]::Escape($repoRoot)) { $aliasDrive = $candidate; break }
            continue
        }
        & subst $candidate $repoRoot | Out-Null
        if ($LASTEXITCODE -eq 0) { $aliasDrive = $candidate; break }
    }
    if (-not $aliasDrive) { throw 'Could not map a free drive letter for the whitespace-free alias.' }
    $projDir = Join-Path ($aliasDrive + '\') $projRel
    Write-Host "note: repo path contains a space -> building via alias $aliasDrive => $repoRoot" -ForegroundColor DarkGray
}

Push-Location $projDir
try {
    # ---- pick the ESP32's own USB-Serial-JTAG port, not a USB-TTL adapter ----
    # With a USB-TTL adapter also plugged in, PlatformIO's auto-detection picks
    # whichever port it enumerates first and fails with
    # "Invalid head of packet" (it is talking to the wrong chip).
    # The ESP32-C3 native USB always has VID 0x303A, so we select by VID.
    $extra = @()
    $needsPort = ($PioArgs -contains 'upload') -or ($PioArgs -contains 'monitor') -or
                 ($PioArgs -contains '-t' -and $PioArgs -contains 'upload')
    if ($needsPort -and -not ($PioArgs -contains '--upload-port') -and
        -not ($PioArgs -contains '-p') -and -not ($PioArgs -contains '--port')) {
        $pyExe = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\python.exe'
        if (Test-Path $pyExe) {
            $detected = & $pyExe -c "import serial.tools.list_ports as lp; p=[x.device for x in lp.comports() if x.vid==0x303A]; print(p[0] if p else '')" 2>$null
            $detected = "$detected".Trim()
            if ($detected) {
                Write-Host "note: ESP32 USB-Serial-JTAG detected on $detected (VID 303A)" -ForegroundColor DarkGray
                $extra = @('--upload-port', $detected)
            } else {
                Write-Warning 'no ESP32 USB-Serial-JTAG (VID 303A) found -- is the board plugged in?'
            }
        }
    }

    & $pioExe @PioArgs @extra
    $code = $LASTEXITCODE
} finally {
    Pop-Location
}
exit $code
