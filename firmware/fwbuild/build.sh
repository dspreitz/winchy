#!/usr/bin/env bash
# Winchy custom MicroPython build (runs INSIDE the espressif/idf container).
#
# Produces a stock ESP32_GENERIC_S3 v1.28.0 firmware with two changes:
#   1. deflate (gzip/zlib) COMPRESSION enabled  -> real gzipped log uploads
#   2. USB product string renamed to "winchy-rope" -> identifiable COM port
#
# The app is NOT frozen into the firmware (kept on the littlefs filesystem),
# per request. Output: /work/out/winchy-rope-<ref>-deflate.bin
set -euo pipefail

MPY_REF="${MPY_REF:-v1.28.0}"
BOARD="${BOARD:-ESP32_GENERIC_S3}"
WORK=/work
SRC="$WORK/micropython"
OUT="$WORK/out"
mkdir -p "$OUT"

if [ ! -d "$SRC/.git" ]; then
    echo ">> cloning MicroPython $MPY_REF"
    git clone --depth 1 -b "$MPY_REF" \
        https://github.com/micropython/micropython.git "$SRC"
fi
cd "$SRC"

echo ">> building mpy-cross"
make -C mpy-cross

cd ports/esp32
echo ">> fetching esp32 submodules for $BOARD"
make BOARD="$BOARD" submodules

echo ">> applying Winchy config"
BOARD_H="boards/$BOARD/mpconfigboard.h"
PORT_H="mpconfigport.h"

# (1) deflate compression. MICROPY_PY_DEFLATE_COMPRESS is #ifndef-guarded in
# py/mpconfig.h, and the board header is included before those defaults, so a
# board-level define wins. Idempotent.
if ! grep -q "MICROPY_PY_DEFLATE_COMPRESS" "$BOARD_H"; then
    cat >> "$BOARD_H" <<'EOF'

// --- Winchy custom build: enable deflate (gzip/zlib) compression ---
#define MICROPY_PY_DEFLATE_COMPRESS (1)
EOF
fi

# (2) USB descriptor strings. These are set unconditionally in the esp32
# mpconfigport.h, so override by replacing the literals. Idempotent (no-op once
# replaced).
sed -i 's/"Espressif Device"/"winchy-rope"/g' "$PORT_H"
sed -i 's/"Espressif Systems"/"Winchy"/g' "$PORT_H"

echo ">> config check:"
grep -n "MICROPY_PY_DEFLATE_COMPRESS" "$BOARD_H" || true
grep -n "winchy-rope\|Winchy" "$PORT_H" || true

echo ">> building firmware ($BOARD)"
make BOARD="$BOARD"

DEST="$OUT/winchy-rope-$MPY_REF-deflate.bin"
cp "build-$BOARD/firmware.bin" "$DEST"
echo ">> DONE -> out/$(basename "$DEST")  ($(stat -c%s "$DEST") bytes)"
