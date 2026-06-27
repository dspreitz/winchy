#!/usr/bin/env bash
# Winchy custom MicroPython build (runs INSIDE the espressif/idf container).
#
# Usage: build.sh <rope|winch>
#
# Produces an ESP32_GENERIC_S3 v1.28.0 firmware for the given role with:
#   1. deflate (gzip/zlib) COMPRESSION enabled   -> gzipped log uploads
#   2. USB product string "winchy-<role>"        -> identifiable COM port
#   3. the role's APPLICATION FROZEN into the image (manifest_<role>.py)
#
# secrets.py + calibration.cal are never frozen; they stay on the filesystem.
# Output: <repo>/_fwbuild/out/winchy-<role>-<ref>.bin
set -euo pipefail

ROLE="${1:-${ROLE:-}}"
if [ "$ROLE" != "rope" ] && [ "$ROLE" != "winch" ]; then
    echo "usage: build.sh <rope|winch>"; exit 2
fi

# Repo root: prefer the env (set by run.ps1 / CI), else derive from this script.
REPO="${WINCHY_REPO:-$(cd "$(dirname "$0")/../.." && pwd)}"
export WINCHY_REPO="$REPO"

MPY_REF="${MPY_REF:-v1.28.0}"
BOARD="${BOARD:-ESP32_GENERIC_S3}"
WORK="${WORK:-$REPO/_fwbuild}"
SRC="$WORK/micropython"
OUT="$WORK/out"
MANIFEST="$REPO/firmware/fwbuild/manifest_${ROLE}.py"
mkdir -p "$OUT"

echo ">> role=$ROLE  repo=$REPO  mpy=$MPY_REF  board=$BOARD"

if [ ! -d "$SRC/.git" ]; then
    echo ">> cloning MicroPython $MPY_REF"
    git clone --depth 1 -b "$MPY_REF" \
        https://github.com/micropython/micropython.git "$SRC"
fi
cd "$SRC"
# Reset the two files we patch to pristine, so per-role edits never accumulate
# when both roles are built from the same clone.
git checkout -- ports/esp32/mpconfigport.h \
    "ports/esp32/boards/$BOARD/mpconfigboard.h" 2>/dev/null || true

echo ">> building mpy-cross"
make -C mpy-cross >/dev/null

cd ports/esp32
# Clean the board build dir so each role builds its own frozen content from
# scratch (the standard single-build layout; isolates rope vs winch in one
# clone). A non-default BUILD= dir breaks the frozen mpy-cross link step.
echo ">> clean build dir"
rm -rf "build-$BOARD"
echo ">> fetching esp32 submodules"
make BOARD="$BOARD" submodules >/dev/null

echo ">> applying Winchy config (deflate + USB name=winchy-$ROLE)"
BOARD_H="boards/$BOARD/mpconfigboard.h"
if ! grep -q "MICROPY_PY_DEFLATE_COMPRESS" "$BOARD_H"; then
    cat >> "$BOARD_H" <<'EOF'

// --- Winchy custom build: enable deflate (gzip/zlib) compression ---
#define MICROPY_PY_DEFLATE_COMPRESS (1)
EOF
fi
sed -i "s/\"Espressif Device\"/\"winchy-$ROLE\"/g" mpconfigport.h
sed -i 's/"Espressif Systems"/"Winchy"/g' mpconfigport.h

echo ">> building firmware (frozen manifest: $(basename "$MANIFEST"))"
make BOARD="$BOARD" FROZEN_MANIFEST="$MANIFEST"

DEST="$OUT/winchy-$ROLE-$MPY_REF.bin"
cp "build-$BOARD/firmware.bin" "$DEST"
echo ">> DONE -> out/$(basename "$DEST")  ($(stat -c%s "$DEST") bytes)"
