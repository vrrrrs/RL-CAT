# RL-CAT Keil project source-list checker
#
# Why this exists:
#   CubeMX REWRITES stm32.uvprojx on every code regeneration
#   (ProjectManager.DeletePrevious = true). File entries we add by hand can be
#   lost, which shows up either as a link error or as silently-not-compiled code.
#
#   This script compares the source list inside .uvprojx against the files that
#   actually exist on disk and reports both directions:
#     MISSING   -- listed in the project but not on disk (link error waiting)
#     UNTRACKED -- on disk but NOT in the project (silently not compiled!)
#
# Usage (run after EVERY CubeMX code regeneration):
#   powershell -ExecutionPolicy Bypass -File firmware/tools/check-uvprojx.ps1
#
# Exit code: 0 = consistent; 1 = inconsistencies found

[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projRoot  = (Resolve-Path (Join-Path $scriptDir '..\stm32')).Path
$uvprojx   = Join-Path $projRoot 'MDK-ARM\stm32.uvprojx'

if (-not (Test-Path $uvprojx)) { throw "project file not found: $uvprojx" }

# ---- 1. parse the source list out of .uvprojx -------------------------------
# NOTE: use local-name() instead of //File -- the .uvprojx root carries an xsi
# namespace and plain //File does not always match.
$raw = Get-Content -Path $uvprojx -Raw
$xml = [xml]$raw

# ---- 1b. STRUCTURAL check ---------------------------------------------------
# Why this exists (learned the hard way):
#   uVision/CubeMX can write a .uvprojx that is *well-formed XML* but has lost
#   the whole <TargetArmAds> block (ARM compiler/assembler/linker options).
#   Symptoms: the Options dialog ("magic wand") does not open, the project
#   cannot be built or flashed from the GUI, and uVision may crash on open.
#   A pure file-list check CANNOT catch this -- the Groups section stays intact.
#   So verify the required blocks and a few device fields every time.
$reqBlocks = @(
    'TargetCommonOption', 'CommonProperty', 'DllOption', 'DebugOption',
    'Utilities', 'TargetArmAds', 'ArmAdsMisc', 'Cads', 'Aads', 'LDads', 'Groups'
)
$structErrors = @()
foreach ($b in $reqBlocks) {
    $n = $xml.SelectNodes("//*[local-name()='$b']")
    if ($n.Count -eq 0) {
        $structErrors += "missing block: <$b>"
    }
}

# Field values that must not be empty (device / flash-download plumbing).
$TO = "//*[local-name()='TargetOption']"
$reqFields = @(
    @('TargetCommonOption/Device',        'device name'),
    @('TargetCommonOption/Cpu',           'memory layout + clock'),
    @('TargetCommonOption/PackID',        'CMSIS pack (must match installed DFP!)'),
    @('TargetCommonOption/FlashDriverDll','flash algorithm'),
    @('TargetCommonOption/RegisterFile',  'device header'),
    @('TargetCommonOption/SFDFile',       'SVD file')
)
foreach ($pair in $reqFields) {
    $xp = $TO
    foreach ($seg in $pair[0].Split('/')) { $xp += "/*[local-name()='$seg']" }
    $n = $xml.SelectSingleNode($xp)
    if ($null -eq $n) {
        $structErrors += "missing field: $($pair[0]) ($($pair[1]))"
    } elseif ([string]::IsNullOrWhiteSpace($n.InnerText)) {
        $structErrors += "EMPTY field: $($pair[0]) ($($pair[1]))"
    }
}

# Compiler selection: <uAC6> must be 0 (AC5) or 1 (AC6).
$uac6 = $xml.SelectSingleNode("//*[local-name()='uAC6']")
if ($null -ne $uac6) {
    $v = $uac6.InnerText.Trim()
    if (($v -ne '0') -and ($v -ne '1')) {
        $structErrors += "<uAC6> = '$v' (must be 0 or 1; 1 = ARM Compiler 6)"
    }
} else {
    $structErrors += "missing <uAC6> (compiler selection)"
}

if ($structErrors.Count -gt 0) {
    Write-Host ""
    Write-Host "PROJECT STRUCTURE PROBLEMS ($($structErrors.Count)):" -ForegroundColor Red
    foreach ($e in $structErrors) { Write-Host "  $e" -ForegroundColor Red }
    Write-Host ""
    Write-Host "The Options dialog will not open and the GUI cannot build/flash." -ForegroundColor Yellow
    Write-Host "Fix: re-create the target in uVision, or restore a known-good .uvprojx." -ForegroundColor Yellow
} else {
    Write-Host "OK: required compiler/device blocks and fields are present."
}

$fileNodes = @($xml.SelectNodes("//*[local-name()='File']"))
if ($fileNodes.Count -eq 0) {
    Write-Host "WARNING: no <File> nodes parsed from .uvprojx" -ForegroundColor Yellow
}

$listed = New-Object System.Collections.Generic.HashSet[string]
foreach ($f in $fileNodes) {
    $fpNode = $f.SelectSingleNode("*[local-name()='FilePath']")
    if ($null -eq $fpNode) { continue }
    $fp = $fpNode.InnerText
    if ([string]::IsNullOrWhiteSpace($fp)) { continue }
    # skip macro paths such as $$Device:...
    if ($fp.StartsWith('$')) { continue }
    $full = Join-Path (Join-Path $projRoot 'MDK-ARM') $fp
    $resolved = [System.IO.Path]::GetFullPath($full)
    [void]$listed.Add($resolved)
}

# ---- 2. files that should be compiled --------------------------------------
# Only the directories we own; HAL/CMSIS are managed by CubeMX.
$scanDirs = @(
    (Join-Path $projRoot 'Core\Src'),
    (Join-Path $projRoot 'Middlewares\SCServo')
)

$onDisk = @()
foreach ($d in $scanDirs) {
    if (-not (Test-Path $d)) { continue }
    $onDisk += Get-ChildItem $d -Filter *.c -File | ForEach-Object { $_.FullName }
    $onDisk += Get-ChildItem $d -Filter *.s -File | ForEach-Object { $_.FullName }
}

Write-Host "== Keil project source-list check ==" -ForegroundColor Cyan
Write-Host ("project : {0}" -f $uvprojx)
Write-Host ("listed  : {0} file entries" -f $listed.Count)
Write-Host ("on disk : {0} files in Core/Src + Middlewares/SCServo" -f $onDisk.Count)
Write-Host ""

$problems = 0

# structural problems count too -- a missing <TargetArmAds> is just as fatal
# as a missing source file, only harder to notice.
$problems += $structErrors.Count

# ---- 3. listed but not on disk ---------------------------------------------
$missing = @()
foreach ($l in $listed) {
    if ($l -notmatch '\\Core\\Src\\' -and $l -notmatch '\\Middlewares\\') { continue }
    if (-not (Test-Path $l)) { $missing += $l }
}
if ($missing.Count -gt 0) {
    $problems += $missing.Count
    Write-Host "MISSING (listed in project, not on disk):" -ForegroundColor Red
    foreach ($m in $missing) { Write-Host ("  - {0}" -f $m.Replace($projRoot, '.')) }
}

# ---- 4. on disk but not listed ---------------------------------------------
$untracked = @()
foreach ($f in $onDisk) {
    if (-not $listed.Contains($f)) { $untracked += $f }
}
if ($untracked.Count -gt 0) {
    $problems += $untracked.Count
    Write-Host "UNTRACKED (on disk, NOT compiled by the Keil project):" -ForegroundColor Yellow
    foreach ($u in $untracked) {
        Write-Host ("  - {0}" -f $u.Replace($projRoot, '.'))
    }
    Write-Host ""
    Write-Host "  FIX: add these files to the 'Application/RL-CAT' group in uVision," -ForegroundColor Yellow
    Write-Host "       then re-run this check. (Or restore the entries in .uvprojx.)" -ForegroundColor Yellow
}

if ($problems -eq 0) {
    Write-Host "OK: project source list is consistent with disk." -ForegroundColor Green
    exit 0
}

Write-Host ""
Write-Host ("INCONSISTENT: {0} problem(s)" -f $problems) -ForegroundColor Red
exit 1
