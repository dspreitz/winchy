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

# Clean vs incremental. A role switch (or CLEAN=1, or no warm build dir) forces a
# clean build so per-role frozen content + config never mix. Rebuilding the SAME
# role keeps the warm build dir AND the config patches, so only the changed app
# re-freezes (~30-60 s) instead of the whole tree. CI always starts from a fresh
# checkout (no _fwbuild / no .last_role), so CI stays a clean build - unchanged.
LAST_ROLE_FILE="$WORK/.last_role"
LAST_ROLE="$(cat "$LAST_ROLE_FILE" 2>/dev/null || echo '')"
INCREMENTAL=0
if [ "${CLEAN:-0}" != "1" ] && [ "$LAST_ROLE" = "$ROLE" ] \
        && [ -d "ports/esp32/build-$BOARD" ]; then
    INCREMENTAL=1
fi

echo ">> building mpy-cross"
make -C mpy-cross >/dev/null

cd ports/esp32
if [ "$INCREMENTAL" = "1" ]; then
    echo ">> INCREMENTAL build (same role '$ROLE'): keeping build-$BOARD + config"
else
    echo ">> CLEAN build (role='$ROLE' last='$LAST_ROLE' CLEAN=${CLEAN:-0})"
    # Reset the two patched files to pristine so per-role edits never accumulate;
    # a non-default BUILD= dir breaks the frozen mpy-cross link, so use the
    # standard single-build layout and wipe it.
    git checkout -- mpconfigport.h "boards/$BOARD/mpconfigboard.h" 2>/dev/null || true
    # Wipe the board build dir. On Docker Desktop's Windows bind mount, rm -rf
    # intermittently fails with "Directory not empty" (grpcfuse sync lag), so
    # retry a few times before giving up (a dirty dir would mix rope/winch).
    n=0
    until rm -rf "build-$BOARD"; do
        n=$((n + 1))
        if [ "$n" -ge 5 ]; then
            echo ">> ERROR: cannot remove build-$BOARD after $n tries (Docker/Windows"
            echo ">>   bind-mount flake). Remove it from the host and re-run:"
            echo ">>   Remove-Item -Recurse -Force _fwbuild/micropython/ports/esp32/build-$BOARD"
            exit 1
        fi
        echo ">> rm build-$BOARD failed (try $n); retrying in 1s..."
        sleep 1
    done
    echo ">> fetching esp32 submodules"
    make BOARD="$BOARD" submodules >/dev/null
    echo ">> applying Winchy config (deflate + USB-Serial-JTAG console)"
    BOARD_H="boards/$BOARD/mpconfigboard.h"
    if ! grep -q "MICROPY_PY_DEFLATE_COMPRESS" "$BOARD_H"; then
        cat >> "$BOARD_H" <<'EOF'

// --- Winchy custom build: enable deflate (gzip/zlib) compression ---
#define MICROPY_PY_DEFLATE_COMPRESS (1)

// --- Winchy custom build: console on the HARDWARE USB-Serial-JTAG ---
// TinyUSB CDC could never emit Guru-Meditation/panic output (the panic
// handler cannot run the TinyUSB stack - every C-level crash was invisible
// until a UART adapter was clamped on), and its CDC wedged repeatedly
// (writes rejected, raw-REPL corruption). The S3's fixed-function
// USB-Serial-JTAG needs no firmware stack: ROM boot banner, IDF logs AND
// panic dumps all arrive over plain USB, esptool can enter the bootloader
// via its reset protocol (no more gentle-REPL machine.bootloader() dance),
// and there is no device-side CDC state left to wedge. UART0 stays enabled
// as a second REPL/console (MICROPY_HW_ENABLE_UART_REPL above), so the
// GPIO43 clamp keeps working as a backstop. Enumeration note: the port is
// now VID 0x303A PID 0x1001 with the MAC (colon form) as serial - the
// same identity in the ROM bootloader and the running app.
#define MICROPY_HW_ENABLE_USBDEV (0)
#define MICROPY_HW_USB_CDC (0)
#define MICROPY_HW_ESP_USB_SERIAL_JTAG (1)
EOF
    fi
    # (TinyUSB product-string seds retired with the CDC itself: the JTAG
    # interface has fixed descriptors; boards are identified by MAC serial.)
fi

echo ">> building firmware (frozen manifest: $(basename "$MANIFEST"), user C modules)"
make BOARD="$BOARD" FROZEN_MANIFEST="$MANIFEST" \
     USER_C_MODULES="$REPO/firmware/cmodules/micropython.cmake"
echo "$ROLE" > "$LAST_ROLE_FILE"   # record role for the next incremental decision

DEST="$OUT/winchy-$ROLE-$MPY_REF.bin"
cp "build-$BOARD/firmware.bin" "$DEST"
echo ">> DONE -> out/$(basename "$DEST")  ($(stat -c%s "$DEST") bytes)"
