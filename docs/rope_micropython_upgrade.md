# Rope MicroPython upgrade: v1.23.0 → v1.28.0 (ESP32-S3 Generic, stable)

Bench procedure for re-flashing the **rope** unit (LilyGo T-Beam S3 Supreme).
Do **not** touch the winch during this — it stays on its own port.

## Why / what
- The rope has **no board-specific MicroPython build**; the correct one is the
  **Generic ESP32-S3** build, which it already runs.
- Current: **v1.23.0** (Jun 2024). Target: **v1.28.0** stable (6 Apr 2026) —
  ~22 months of fixes, newer asyncio, newer ESP-IDF, possibly better native
  USB-CDC (the thing that wedges `mpremote` deploys).
- The rope's current build already maps the **full 8 MB PSRAM** (heap shows an
  `8388608` region). The upgrade only has to *preserve* that — see step 5.

> **Re-flashing ERASES the entire littlefs filesystem.** The app code lives in
> git and is re-deployed, but the **device-only** files are not in git and must
> be backed up first (step 1). Skipping this loses the WiFi/GitHub secrets and
> the force calibration.

Port assumption below: **rope = COM10** (verify with Device Manager; the winch
was COM6). Replace `COM10` if it has moved.

---

## 0. Prerequisites (on the PC)
```powershell
python -m esptool version      # install if missing:  pip install esptool
python -m mpremote --help      # already installed
```
Keep the rope on **USB power** for the whole procedure (don't run it off the
cell). Download both firmware files into a known folder, e.g. `C:\fw\`:

- Standard:    https://micropython.org/resources/firmware/ESP32_GENERIC_S3-20260406-v1.28.0.bin
- Octal-SPIRAM: https://micropython.org/resources/firmware/ESP32_GENERIC_S3-SPIRAM_OCT-20260406-v1.28.0.bin

Also keep a **rollback** copy of the current line — download
`ESP32_GENERIC_S3-…-v1.23.0.bin` from the v1.23.0 archive on micropython.org so
you can revert if a driver misbehaves.

---

## 1. Back up the device-only files (CRITICAL)
These are **not in git** and are wiped by the erase. Interrupt the app first so
the radio/console prints don't corrupt the transfer.

```powershell
# stop the running app (Ctrl-C into the REPL); mpremote does this for exec/cp,
# but do an explicit soft-reset-to-REPL to be safe:
python -m mpremote connect COM10 exec "import sys"   # forces raw-REPL interrupt

mkdir C:\rope_fs_backup
cd C:\rope_fs_backup
python -m mpremote connect COM10 fs cp :secrets.py .         # WiFi + GitHub PAT + ZTP token  (CRITICAL)
python -m mpremote connect COM10 fs cp :calibration.cal .    # force tare/calibration          (CRITICAL)
python -m mpremote connect COM10 fs cp :last_fix.json .       # QNH seed (regenerates)
python -m mpremote connect COM10 fs cp :mga_chip.json .       # AssistNow ZTP chipcode (saves re-register)
python -m mpremote connect COM10 fs cp :mga_offline.ubx .     # AssistNow orbit cache
python -m mpremote connect COM10 fs cp :mga_offline.ts .
python -m mpremote connect COM10 fs cp :crash.log .           # diagnostic (optional)
python -m mpremote connect COM10 fs cp :raw.csv .             # the raw log (large; or upload to GitHub first)
```
Confirm `secrets.py` and `calibration.cal` are present and non-empty before
going further. **If raw.csv matters, upload it to winchy-logs now** (it's gone
after the erase).

---

## 2. Enter the ROM download mode
Cleanest method — no buttons needed. This reboots the chip straight into the
serial bootloader:
```powershell
python -m mpremote connect COM10 exec "import machine; machine.bootloader()"
```
The USB **re-enumerates** as the ROM USB-Serial-JTAG (VID 303A **PID 1001**),
possibly under a **different COM number**. Find it:
```powershell
Get-CimInstance Win32_PnPEntity | ? { $_.Name -match 'COM\d+' } | ft Name, DeviceID -Auto -Wrap
```
Use that port below as `<DLPORT>` (it may still be COM10, or e.g. COM11).

*Fallback if `machine.bootloader()` is unreachable:* hold the **BOOT/IO0**
button, tap **RST**, release BOOT — then re-check the port.

---

## 3. Erase + write firmware
Start with the **standard** variant (it always boots even if PSRAM init is
wrong — safe to probe; step 5 confirms PSRAM and switches to octal only if
needed).
```powershell
python -m esptool --chip esp32s3 --port <DLPORT> erase_flash
python -m esptool --chip esp32s3 --port <DLPORT> --baud 460800 write_flash -z 0 C:\fw\ESP32_GENERIC_S3-20260406-v1.28.0.bin
```
Offset is **0** (the .bin is a combined image incl. bootloader + partition
table). After it finishes, power-cycle or tap RST; the board comes back as a
MicroPython USB-CDC port again (re-check the COM number).

---

## 4. Verify version + PSRAM
```powershell
python -m mpremote connect COM10 exec "import sys,gc,esp32; gc.collect(); print(sys.implementation); print('FREE',gc.mem_free()); print('HEAP',esp32.idf_heap_info(esp32.HEAP_DATA))"
```
Expect:
- `version=(1, 28, 0, '')`
- An `8388608` region in HEAP and `FREE` ≈ 7–8 MB.

**If PSRAM is missing** (FREE ≈ 300 KB, no 8 MB region) → the board is octal;
re-do steps 2–3 with the **SPIRAM_OCT** file
(`ESP32_GENERIC_S3-SPIRAM_OCT-20260406-v1.28.0.bin`) and re-verify.

---

## 5. Re-deploy the app + restore device-only files
Deploy the code from the repo (`firmware/rope/`), then drop the backed-up
device-only files back.
```powershell
cd C:\Users\dom14\vs_code\Winchy\firmware\rope
python -m mpremote connect COM10 fs cp -r boot.py main.py config.py protocol.py wifi.py lib winchy :

# restore the gitignored device-only files:
cd C:\rope_fs_backup
python -m mpremote connect COM10 fs cp secrets.py :
python -m mpremote connect COM10 fs cp calibration.cal :
python -m mpremote connect COM10 fs cp last_fix.json :
python -m mpremote connect COM10 fs cp mga_chip.json :
python -m mpremote connect COM10 fs cp mga_offline.ubx :
python -m mpremote connect COM10 fs cp mga_offline.ts :
```
Reboot into the app:
```powershell
python -m mpremote connect COM10 reset
```

---

## 6. Driver-by-driver verification (watch the console)
```powershell
python -m mpremote connect COM10 repl     # Ctrl-] to exit
```
Confirm each subsystem on a v1.28.0 build — these are the drivers most likely to
shift on a new MicroPython/IDF:

- [ ] **Boot clean** — no Traceback, no crash-guard reset loop.
- [ ] **AXP2101 PMU** — `PMU: system …mV, battery …mV (…%)` prints; charge LED on.
- [ ] **QMI8658 IMU** — `Motion: |a|=… gyro=…` looks sane (≈1 g at rest).
- [ ] **sh1106 OLED** — display shows force/angle/sats.
- [ ] **Barometer** — `Baro alt:` line; plausible altitude.
- [ ] **GPS (UBX NAV-PVT)** — `Sat:` count climbs, `RTC synced from GPS`, PPS ticks.
- [ ] **ADS1232 force** — `ADC value:` stable; **re-check the bit-banged SCLK +
      `disable_irq` timing** (most timing-sensitive driver — verify counts aren't
      noisy/garbage vs the old build). Re-run a tare if needed.
- [ ] **SX1262 radio** — `Sent packet`; winch receives; link/RSSI reports return.
- [ ] **WiFi + dashboard** — joins, dashboard loads, manual upload works.
- [ ] **Power-off button** — long-press powers off and **stays** off (no auto-restart).

---

## 7. Rollback
If a driver can't be made to work, revert: repeat steps 2–3 with the saved
**v1.23.0** .bin, then re-deploy (step 5). The backup from step 1 restores the
secrets + calibration regardless of version.

---

## Notes / gotchas
- This is the **rope only**. The winch (T3-S3, 2 MB PSRAM) has a *board-specific*
  build but only **preview** versions exist — do it separately, later, if at all.
- `deflate` compression (the gzip-upload bug) is **build-config dependent** and
  likely still absent at the ESP32 feature level — verify on v1.28.0 with
  `import deflate,io; deflate.DeflateIO(io.BytesIO(), deflate.GZIP).write`. If it
  raises `AttributeError`, the raw-CSV upload fallback stays in effect (no
  regression).
- Native-USB ESP32-S3 sometimes needs the port re-selected after every reset —
  always re-check the COM number rather than assuming it stayed put.
