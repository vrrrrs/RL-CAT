# RL-CAT STM32 firmware build (armclang / Arm Compiler 6)
#
# Why this script exists:
#   This machine has MDK 5.40 with AC6.22 installed, but the CubeMX-generated
#   .uvprojx selects AC5 ("Default Compiler Version 5"), so `UV4 -b` aborts.
#   The compiler selection does not live in .uvprojx/.uvoptx, so it cannot be
#   patched from a script. We therefore build with the SAME compiler, same
#   target, same macros and include paths as the Keil project -- but fully on
#   the command line, which also gives us Flash/RAM size reporting.
#
# Usage (from firmware/stm32):
#   powershell -ExecutionPolicy Bypass -File ../tools/build.ps1
#
# Exit code: 0 = ok (0 error, 0 warning); 1 = build failed; 2 = warnings only

[CmdletBinding()]
param(
    [string]$KeilRoot = 'D:\Software\Work\Keil',
    [switch]$KeepObjects,
    [switch]$AllowWarnings
)

$ErrorActionPreference = 'Stop'

$toolBin  = Join-Path $KeilRoot 'ARM\ARMCLANG\bin'
$armclang = Join-Path $toolBin 'armclang.exe'
$armlink  = Join-Path $toolBin 'armlink.exe'
$fromelf  = Join-Path $toolBin 'fromelf.exe'

foreach ($t in @($armclang, $armlink, $fromelf)) {
    if (-not (Test-Path $t)) { throw "Toolchain not found: $t" }
}

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projRoot  = (Resolve-Path (Join-Path $scriptDir '..\stm32')).Path
$buildDir  = Join-Path $projRoot 'build'
$sct       = Join-Path $scriptDir 'rlcat_f103.sct'

if (Test-Path $buildDir) { Remove-Item $buildDir -Recurse -Force }
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

# ---- target and language -----------------------------------------------
$target  = @('--target=arm-arm-none-eabi', '-mcpu=cortex-m3')
$defines = @('-DUSE_HAL_DRIVER', '-DSTM32F103xB')

$includes = @(
    (Join-Path $projRoot 'Core\Inc'),
    (Join-Path $projRoot 'Drivers\STM32F1xx_HAL_Driver\Inc'),
    (Join-Path $projRoot 'Drivers\STM32F1xx_HAL_Driver\Inc\Legacy'),
    (Join-Path $projRoot 'Drivers\CMSIS\Device\ST\STM32F1xx\Include'),
    (Join-Path $projRoot 'Drivers\CMSIS\Include')
) | ForEach-Object { "-I$_" }

# NOTE: -fshort-enums matches Keil AC6 defaults. Without it, enum-backed
# struct layouts can differ from a Keil GUI build.
$cflags = $target + @(
    '-std=gnu11',
    '-Os',
    '-g',
    '-fshort-enums',
    '-fno-common',
    '-Wall',
    '-Wextra',
    '-Wno-unused-parameter',
    # ST HAL code has a few `if ((x == y))` spots that trip -Wextra; upstream
    # Keil builds are silent on them, so we suppress rather than edit the HAL.
    '-Wno-parentheses-equality',
    '-ffunction-sections',
    '-fdata-sections'
) + $defines + $includes

# ---- sources -----------------------------------------------------------
function Get-Sources([string]$dir) {
    if (-not (Test-Path $dir)) { return @() }
    Get-ChildItem $dir -Filter *.c -File |
        Where-Object { $_.Name -notlike '*_template.c' } |
        Sort-Object Name | ForEach-Object { $_.FullName }
}

$cSources  = @()
$cSources += Get-Sources (Join-Path $projRoot 'Core\Src')
$cSources += Get-Sources (Join-Path $projRoot 'Middlewares\SCServo')
$cSources += Get-Sources (Join-Path $projRoot 'Drivers\STM32F1xx_HAL_Driver\Src')

if ($cSources.Count -eq 0) { throw 'No .c sources found' }

$startup = Join-Path $projRoot 'MDK-ARM\startup_stm32f103xb.s'
if (-not (Test-Path $startup)) { throw "Startup file not found: $startup" }

Write-Host "== RL-CAT firmware build ==" -ForegroundColor Cyan
Write-Host ("sources : {0} .c + 1 startup .s" -f $cSources.Count)
Write-Host ("out dir : {0}" -f $buildDir)

$objects  = @()
$errors   = 0
$warnings = 0
$logLines = New-Object System.Collections.Generic.List[string]

# Diagnostics we deliberately tolerate:
#   1950W -- armclang falls back to the legacy armasm assembler for the CubeMX
#            startup file (Arm syntax). This is expected and correct.
$script:ignoredDiag = @('A1950W')

function Test-Ignored([string]$s) {
    foreach ($pat in $script:ignoredDiag) { if ($s -match $pat) { return $true } }
    return $false
}

function Invoke-Tool {
    param([string]$Exe, [string[]]$Argv, [string]$Tag)
    # Native tools write diagnostics to stderr; with $ErrorActionPreference='Stop'
    # PowerShell would turn that into a terminating NativeCommandError. We want
    # the exit code instead, so relax it for the duration of the call.
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $out  = & $Exe @Argv 2>&1
        $code = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $prev
    }
    foreach ($line in $out) {
        $s = "$line"
        if ($s.Trim().Length -eq 0) { continue }
        $logLines.Add($s)
        if (Test-Ignored $s) { continue }
        # Match real diagnostics only. armasm prints a summary line
        # ("0 Errors, 1 Warning") which must not be counted as an error.
        if ($s -match 'error\s*:' -or $s -match '^\s*\d+\s+errors?\s+generated') {
            $script:errors++
            Write-Host "  [E] $s" -ForegroundColor Red
        } elseif ($s -match 'warning\s*:' -or $s -match '^\s*\d+\s+warnings?\s+generated') {
            $script:warnings++
            Write-Host "  [W] $s" -ForegroundColor Yellow
        }
    }
    if ($code -ne 0 -and $script:errors -eq 0) {
        $script:errors++
        Write-Host "  [E] $Tag exited with code $code" -ForegroundColor Red
    }
    return $code
}

# ---- compile -----------------------------------------------------------
$i = 0
foreach ($src in $cSources) {
    $i++
    $name = [System.IO.Path]::GetFileNameWithoutExtension($src)
    $obj  = Join-Path $buildDir "$name.o"
    Write-Progress -Activity 'Compiling' -Status $name -PercentComplete (100 * $i / ($cSources.Count + 1))
    Invoke-Tool -Exe $armclang -Argv (@('-c') + $cflags + @('-o', $obj, $src)) -Tag $name | Out-Null
    if (Test-Path $obj) { $objects += $obj }
}

Write-Progress -Activity 'Compiling' -Status 'startup' -PercentComplete 100
$startupObj = Join-Path $buildDir 'startup_stm32f103xb.o'
# The CubeMX startup file uses Arm legacy assembler syntax (AREA/SPACE/DCD/
# EXPORT/PROC), which is armasm's dialect -- NOT GNU as. armclang's integrated
# assembler needs -masm=auto to accept it.
$asmFlags = $target + @('-masm=auto', '-x', 'assembler-with-cpp')
Invoke-Tool -Exe $armclang -Argv (@('-c') + $asmFlags + @('-o', $startupObj, $startup)) -Tag 'startup' | Out-Null
if (Test-Path $startupObj) { $objects += $startupObj }
Write-Progress -Activity 'Compiling' -Completed

if ($errors -gt 0) {
    Write-Host ""
    Write-Host "COMPILE FAILED: $errors error(s), $warnings warning(s)" -ForegroundColor Red
    $logLines | Set-Content -Path (Join-Path $buildDir 'build.log') -Encoding UTF8
    exit 1
}

# ---- link --------------------------------------------------------------
$axf = Join-Path $buildDir 'stm32.axf'
$linkArgv = @(
    '--cpu=Cortex-M3',
    '--scatter', $sct,
    '--strict',
    '--info=sizes',
    '-o', $axf
) + $objects
Invoke-Tool -Exe $armlink -Argv $linkArgv -Tag 'link' | Out-Null

if (-not (Test-Path $axf)) {
    Write-Host ""
    Write-Host "LINK FAILED" -ForegroundColor Red
    $logLines | Set-Content -Path (Join-Path $buildDir 'build.log') -Encoding UTF8
    exit 1
}

# ---- artifacts ---------------------------------------------------------
$hex = Join-Path $buildDir 'stm32.hex'
$bin = Join-Path $buildDir 'stm32.bin'
Invoke-Tool -Exe $fromelf -Argv @('--i32combined', '-o', $hex, $axf) -Tag 'hex' | Out-Null
Invoke-Tool -Exe $fromelf -Argv @('--bin', '-o', $bin, $axf)          -Tag 'bin' | Out-Null

$prevEap = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
$sizeOut = & $fromelf --text -z $axf 2>&1
$ErrorActionPreference = $prevEap
foreach ($l in $sizeOut) { $logLines.Add("$l") }

# ---- report ------------------------------------------------------------
Write-Host ""
Write-Host "== BUILD RESULT ==" -ForegroundColor Cyan

# The -z report has a header row, then one data row per object, then a final
# "ROM Totals" row. The FIRST data row (5 columns present, not "ROM Totals")
# already carries the whole-image totals, so we take that one.
$dataRows = @($sizeOut | Where-Object { $_ -match '^\s+\d+\s+\d+\s+\d+\s+\d+\s+\d+\s' })
$img = $null
foreach ($row in $dataRows) {
    if ($row -notmatch 'ROM Totals') { $img = $row; break }
}
if ($null -ne $img) {
    $f = ($img -split '\s+') | Where-Object { $_ -ne '' }
    #  f[0]=Code f[1]=(inc.data) f[2]=RO Data f[3]=RW Data f[4]=ZI Data f[5]=Debug
    $rom = [int]$f[0] + [int]$f[2] + [int]$f[3]
    $ram = [int]$f[3] + [int]$f[4]
    Write-Host ("  Code {0} B   RO Data {1} B   RW Data {2} B   ZI Data {3} B" -f `
                $f[0], $f[2], $f[3], $f[4])
    Write-Host ""
    Write-Host ("  FLASH : {0} / 65536 B  ({1:N1}%)   free {2} B" -f `
                $rom, (100.0 * $rom / 65536), (65536 - $rom))
    Write-Host ("  RAM   : {0} / 20480 B  ({1:N1}%)   free {2} B" -f `
                $ram, (100.0 * $ram / 20480), (20480 - $ram))

    # Budget guard (docs/33-firmware-plan.md section 6: Flash 64 KB, RAM 20 KB).
    # RAM limit leaves 2 KB headroom for the stack.
    if ($rom -gt 65536) {
        Write-Host "  !! FLASH OVERFLOW" -ForegroundColor Red
        $script:errors++
    }
    if ($ram -gt 18432) {
        Write-Host "  !! RAM NEARLY EXHAUSTED" -ForegroundColor Red
        $script:errors++
    }
} else {
    Write-Host "  (could not parse size report)" -ForegroundColor Yellow
}

$binSize = if (Test-Path $bin) { (Get-Item $bin).Length } else { 0 }
Write-Host ""
Write-Host ("  HEX : {0}" -f $hex)
Write-Host ("  BIN : {0}  ({1} bytes)" -f $bin, $binSize)
Write-Host ("  errors {0}   warnings {1}" -f $errors, $warnings)

$logPath = Join-Path $buildDir 'build.log'
$logLines | Set-Content -Path $logPath -Encoding UTF8
Write-Host ("  log : {0}" -f $logPath)

if (-not $KeepObjects) {
    Get-ChildItem $buildDir -Filter *.o | Remove-Item -Force
}

if ($errors -gt 0) { exit 1 }
if ($warnings -gt 0 -and -not $AllowWarnings) {
    Write-Host ""
    Write-Host "WARNINGS PRESENT ($warnings) -- pass -AllowWarnings to ignore" -ForegroundColor Yellow
    exit 2
}
Write-Host ""
Write-Host "BUILD OK (0 error, 0 warning)" -ForegroundColor Green
exit 0
