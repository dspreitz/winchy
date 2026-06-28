# Winchy custom MicroPython build - host launcher (Windows / Docker Desktop).
#
# Usage:  .\firmware\fwbuild\run.ps1 rope
#         .\firmware\fwbuild\run.ps1 winch
#         .\firmware\fwbuild\run.ps1 all          (default: builds both)
#         .\firmware\fwbuild\run.ps1 rope -Clean  (force a full rebuild)
#
# Mounts the repo at /repo and runs build.sh inside espressif/idf. The
# MicroPython clone and the output .bin land in <repo>\_fwbuild (gitignored).
# Rebuilding the same role is INCREMENTAL (warm build-dir kept); a role switch
# or -Clean forces a full rebuild. A persistent ccache volume speeds C compiles
# across runs (incl. clean builds + role switches). CI is unaffected (it always
# checks out fresh, so it has no warm dir / no ccache and always builds clean).
param([string]$Role = 'all', [switch]$Clean)
$ErrorActionPreference = 'Stop'

$scripts = $PSScriptRoot                                    # ...\firmware\fwbuild
$repo    = Split-Path -Parent (Split-Path -Parent $scripts) # ...\Winchy
$image   = 'espressif/idf:v5.5.1'

$roles = if ($Role -eq 'all') { @('rope', 'winch') } else { @($Role) }

New-Item -ItemType Directory -Force -Path (Join-Path $repo '_fwbuild') | Out-Null
Write-Host ">> image : $image"
Write-Host ">> repo  : $repo  -> /repo"
Write-Host ">> roles : $($roles -join ', ')"

$cleanEnv = if ($Clean) { '1' } else { '0' }
foreach ($r in $roles) {
    Write-Host "`n=== building winchy-$r (clean=$cleanEnv) ==="
    # tr -d '\r' guards against CRLF from a Windows checkout breaking bash.
    # winchy-ccache: persistent ccache volume; IDF_CCACHE_ENABLE makes IDF use it.
    docker run --rm `
        -v "${repo}:/repo" `
        -v winchy-ccache:/root/.ccache `
        -e WINCHY_REPO=/repo `
        -e IDF_CCACHE_ENABLE=1 `
        -e CLEAN=$cleanEnv `
        $image bash -c "tr -d '\r' < /repo/firmware/fwbuild/build.sh | bash -s -- $r"
}
Write-Host "`n>> output in $repo\_fwbuild\out"
