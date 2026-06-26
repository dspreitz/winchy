# Winchy custom MicroPython build - host launcher (Windows / Docker Desktop).
# Pulls the espressif/idf image and runs build.sh inside it. The MicroPython
# source clone and the output .bin land in <repo>\_fwbuild (gitignored).
$ErrorActionPreference = 'Stop'

$scripts = $PSScriptRoot                                   # ...\firmware\fwbuild
$repo    = Split-Path -Parent (Split-Path -Parent $scripts) # ...\Winchy
$work    = Join-Path $repo '_fwbuild'
$image   = 'espressif/idf:v5.5.1'

New-Item -ItemType Directory -Force -Path $work | Out-Null

Write-Host ">> image : $image"
Write-Host ">> work  : $work"
Write-Host ">> output: $work\out"

docker run --rm `
    -v "${scripts}:/scripts" `
    -v "${work}:/work" `
    $image bash /scripts/build.sh
