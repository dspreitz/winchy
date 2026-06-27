# Custom MicroPython firmware build (Docker)

Builds a custom **ESP32_GENERIC_S3 v1.28.0** image **per role** (`rope`, `winch`)
with the application **frozen into the firmware**:

1. **`deflate` compression enabled** (`MICROPY_PY_DEFLATE_COMPRESS`) — gzipped
   log uploads instead of raw CSV.
2. **USB product string `winchy-<role>`** — the COM port is identifiable.
3. **App frozen** via `manifest_<role>.py` — the whole application ships inside
   the `.bin`; no `mpremote` deploy of app code needed.

**Stays on the device filesystem (never frozen):** `secrets.py` (WiFi/GitHub/ZTP
secrets — must never end up in a public binary) and `calibration.cal` (per-device
force calibration), plus the tiny boot glue (`boot.py`/`main.py`).

## Layout
- `build.sh <rope|winch>` — runs in the container: clone MicroPython v1.28.0,
  reset+patch config (deflate + USB name), build with the role manifest.
- `manifest_rope.py` / `manifest_winch.py` — what gets frozen for each role.
- `run.ps1 [rope|winch|all]` — Windows launcher; pulls `espressif/idf:v5.5.1`
  and runs `build.sh` with the repo mounted at `/repo`.
- Output (gitignored): `_fwbuild/out/winchy-<role>-v1.28.0.bin`.

## Build locally
```powershell
# Docker Desktop must be running
.\firmware\fwbuild\run.ps1 all      # or: rope / winch
```
First run downloads the IDF image (~GBs) + clones/compiles; re-runs reuse the
clone. Both roles build from one clone (the patched config is reset per role).

## CI
`.github/workflows/firmware.yml` does the same on every push/PR:
`test` (host `pytest`) → `build` (matrix rope/winch, the build doubles as the
compile smoke-test) → release. Push to `main` refreshes the rolling **`latest`**
prerelease; a **`v*` tag** cuts a versioned release. Both carry both `.bin`.

## Flash (per role)
BOOT+RST into download mode, then:
```powershell
python -m esptool --chip esp32s3 --port <DLPORT> erase-flash
python -m esptool --chip esp32s3 --port <DLPORT> --baud 460800 write-flash -z 0 `
    _fwbuild\out\winchy-rope-v1.28.0.bin
```
After flashing a **frozen** image the filesystem only needs:
- **rope:** `boot.py`, `main.py`, `secrets.py`, `calibration.cal`
- **winch:** `main.py` (the `import winch_app` launcher), `secrets.py`

(See `docs/rope_micropython_upgrade.md` for the full download-mode/backup dance.)

## Verify
```python
import deflate, io
b = io.BytesIO(); d = deflate.DeflateIO(b, deflate.GZIP); d.write(b'x'*600); d.close()
print(len(b.getvalue()))           # succeeds -> compression present
import winchy            # rope: frozen app importable
```
The COM port shows up named **winchy-rope** / **winchy-winch**.
