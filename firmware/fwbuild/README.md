# Custom MicroPython firmware build (Docker)

Builds a custom **ESP32_GENERIC_S3 v1.28.0** firmware for the rope unit with:

1. **`deflate` compression enabled** (`MICROPY_PY_DEFLATE_COMPRESS`) — fixes the
   gzip-upload path so logs upload compressed instead of raw CSV.
2. **USB product string = `winchy-rope`** — the COM port is identifiable over USB
   instead of the generic "Serielles USB-Gerät".

The application is **not** frozen into the firmware — it stays on the littlefs
filesystem and is deployed with `mpremote` as today.

## How it works
- `run.ps1` (host) pulls `espressif/idf:v5.5.1` and runs `build.sh` inside it.
- `build.sh` (container) clones MicroPython `v1.28.0`, applies the two config
  changes, and builds.
- The clone and the resulting `.bin` go to `<repo>\_fwbuild\` (gitignored).
  Output: `_fwbuild\out\winchy-rope-v1.28.0-deflate.bin`.

## Run
```powershell
# Docker Desktop must be running
.\firmware\fwbuild\run.ps1
```
First run downloads the IDF image (~GBs) and clones/compiles (several minutes).
Re-runs reuse the clone and are fast.

## Flash
Same as a stock flash (BOOT+RST into download mode):
```powershell
python -m esptool --chip esp32s3 --port <DLPORT> erase-flash
python -m esptool --chip esp32s3 --port <DLPORT> --baud 460800 write-flash -z 0 `
    _fwbuild\out\winchy-rope-v1.28.0-deflate.bin
```
Then re-deploy the app + restore `secrets.py`/`calibration.cal` (see
`docs/rope_micropython_upgrade.md`).

## Verify after flashing
```python
import deflate, io
b = io.BytesIO(); d = deflate.DeflateIO(b, deflate.GZIP); d.write(b'x'*600); d.close()
print(len(b.getvalue()))   # succeeds now (no AttributeError)
```
The COM port should also show up named **winchy-rope**.
