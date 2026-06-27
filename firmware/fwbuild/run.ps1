# Winchy custom MicroPython build - host launcher (Windows / Docker Desktop).
#
# Usage:  .\firmware\fwbuild\run.ps1 rope
#         .\firmware\fwbuild\run.ps1 winch
#         .\firmware\fwbuild\run.ps1 all     (default: builds both)
#
# Mounts the repo at /repo and runs build.sh inside espressif/idf. The
# MicroPython clone and the output .bin land in <repo>\_fwbuild (gitignored).
param([string]$Role = 'all')
$ErrorActionPreference = 'Stop'

$scripts = $PSScriptRoot                                    # ...\firmware\fwbuild
$repo    = Split-Path -Parent (Split-Path -Parent $scripts) # ...\Winchy
$image   = 'espressif/idf:v5.5.1'

$roles = if ($Role -eq 'all') { @('rope', 'winch') } else { @($Role) }

New-Item -ItemType Directory -Force -Path (Join-Path $repo '_fwbuild') | Out-Null
Write-Host ">> image : $image"
Write-Host ">> repo  : $repo  -> /repo"
Write-Host ">> roles : $($roles -join ', ')"

foreach ($r in $roles) {
    Write-Host "`n=== building winchy-$r ==="
    # tr -d '\r' guards against CRLF from a Windows checkout breaking bash.
    docker run --rm `
        -v "${repo}:/repo" `
        -e WINCHY_REPO=/repo `
        $image bash -c "tr -d '\r' < /repo/firmware/fwbuild/build.sh | bash -s -- $r"
}
Write-Host "`n>> output in $repo\_fwbuild\out"
