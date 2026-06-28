# Winchy: build (incremental) + flash one role in a single step.
#
# Usage:  .\firmware\fwbuild\flash.ps1 rope
#         .\firmware\fwbuild\flash.ps1 rope -Port COM4
#         .\firmware\fwbuild\flash.ps1 rope -Clean
#         .\firmware\fwbuild\flash.ps1 rope -NoBuild   (flash the existing .bin)
#
# Builds via run.ps1 (incremental unless -Clean), then esptool write-flash at
# 0x0 WITHOUT erase, so the device filesystem (secrets.py, calibration.cal,
# last_fix.json, the AssistNow cache) is preserved - secrets.py lives nowhere
# else. The target must already be in DOWNLOAD MODE (hold BOOT, tap RST). RTS
# reset is a no-op on USB-JTAG, so after flashing POWER-CYCLE to boot.
param(
    [Parameter(Mandatory = $true)][ValidateSet('rope', 'winch')][string]$Role,
    [string]$Port,
    [switch]$Clean,
    [switch]$NoBuild
)
$ErrorActionPreference = 'Stop'
$scripts = $PSScriptRoot
$repo = Split-Path -Parent (Split-Path -Parent $scripts)

if (-not $NoBuild) {
    Write-Host ">> building winchy-$Role ..."
    & (Join-Path $scripts 'run.ps1') $Role -Clean:$Clean
    if ($LASTEXITCODE -ne 0) { throw "build failed (exit $LASTEXITCODE)" }
}

$bin = Get-ChildItem (Join-Path $repo "_fwbuild\out\winchy-$Role-*.bin") -ErrorAction SilentlyContinue |
    Sort-Object LastWriteTime | Select-Object -Last 1
if (-not $bin) { throw "no firmware .bin in _fwbuild\out for role '$Role' - build first (omit -NoBuild)" }
Write-Host ">> firmware: $($bin.FullName)  ($($bin.Length) bytes, built $($bin.LastWriteTime))"

if (-not $Port) {
    $dm = Get-CimInstance Win32_PnPEntity |
        Where-Object { $_.PNPDeviceID -match 'VID_303A&PID_1001' -and $_.Name -match 'COM\d+' }
    if (-not $dm) {
        throw "no ESP32-S3 in DOWNLOAD MODE found (VID_303A&PID_1001). Hold BOOT, tap RST, then retry."
    }
    $Port = ([regex]::Match($dm[0].Name, 'COM\d+')).Value
    Write-Host ">> auto-detected download-mode port: $Port"
}

Write-Host ">> flashing to $Port (write-flash @ 0x0, NO erase - filesystem preserved) ..."
python -m esptool --chip esp32s3 -p $Port -b 921600 write-flash -z 0x0 $bin.FullName
if ($LASTEXITCODE -ne 0) { throw "esptool failed (exit $LASTEXITCODE)" }

Write-Host "`n>> DONE. RTS reset is a no-op on USB-JTAG - POWER-CYCLE the $Role" `
    "(unplug/replug USB, do NOT hold BOOT) to boot the new firmware."
