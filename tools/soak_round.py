"""Fully automated soak round for the Winchy rope.

Usage: python soak_round.py <git-ref> <soak-minutes>

One round = checkout -> clean build -> software bootloader entry (no BOOT/RST
press) -> esptool flash (no erase) -> reset -> wait for WiFi -> soak watch ->
verdict from reachability + rst= markers. Prints a VERDICT line at the end.
"""
import json
import re
import subprocess
import sys
import time
import urllib.request

import serial
import serial.tools.list_ports

REPO = r"C:\Users\dom14\vs_code\Winchy"


def _arg(name, default):
    for a in sys.argv:
        if a.startswith(name + "="):
            return a.split("=", 1)[1]
    return default


# Board selection (defaults = board 1 / the original rope). For board 2 pass
# e.g.: --mac=48:ca:43:5a:ca:c0 --url=http://<board2-ip>
ROPE_MAC = _arg("--mac", "34:b7:da:63:e6:54")
ROPE_URL = _arg("--url", "http://192.168.178.128")
WINCH_URL = "http://192.168.178.127"
BIN = REPO + r"\_fwbuild\out\winchy-rope-v1.28.0.bin"

GIT_REF = sys.argv[1]
SOAK_MIN = int(sys.argv[2]) if len(sys.argv) > 2 else 45
SKIP_BUILD = "--skip-build" in sys.argv   # bin already built for GIT_REF
SOAK_ONLY = "--soak-only" in sys.argv     # device already runs GIT_REF:
                                          # no checkout/build/flash, just soak


def log(msg):
    print("[%s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


def run(cmd, timeout=600, check=True):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if check and r.returncode != 0:
        log("CMD FAILED: %s\n%s\n%s" % (cmd, (r.stdout or "")[-800:],
                                        (r.stderr or "")[-800:]))
        raise SystemExit("VERDICT: ABORT (command failed)")
    return r


# TinyUSB serial number = chip MAC without colons
ROPE_USB_SERIAL = ROPE_MAC.replace(":", "").upper()


def ports_by_pid(pid):
    return [p.device for p in serial.tools.list_ports.comports()
            if p.vid == 0x303A and p.pid == pid]


def rope_cdc_port():
    # The RUNNING rope is identified by its USB serial number (= MAC) so the
    # bootloader entry can never hit the winch by accident.
    for p in serial.tools.list_ports.comports():
        if (p.vid == 0x303A and p.pid == 0x4001 and p.serial_number
                and p.serial_number.startswith(ROPE_USB_SERIAL)):
            return p.device
    return None


def rope_download_port(timeout_s=30):
    # The ROM bootloader's USB serial number IS the MAC (with colons), so the
    # rope is identified without any esptool probe (an esptool probe with the
    # default post-command hard reset even kicked the chip back OUT of the
    # bootloader - cost one debugging round).
    want = ROPE_MAC.replace(":", "").upper()
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        for p in serial.tools.list_ports.comports():
            sn = (p.serial_number or "").replace(":", "").upper()
            if p.vid == 0x303A and p.pid == 0x1001 and sn.startswith(want):
                return p.device
        time.sleep(2)
    return None


def http_ok(url, timeout=8):
    try:
        urllib.request.urlopen(url + "/data", timeout=timeout).read()
        return True
    except Exception:
        return False


def fetch_rst_markers():
    try:
        raw = urllib.request.urlopen(ROPE_URL + "/raw", timeout=90).read()
        txt = raw.decode("utf-8", "replace")
        boots = txt.count("# boot")
        rst = re.findall(r"rst=([A-Za-z/()0-9]+)", txt)
        return boots, rst
    except Exception as e:
        return None, ["(raw fetch failed: %s)" % type(e).__name__]


def usb_reset():
    # Driver-level "unplug/replug" of the rope's USB device via the elevated
    # scheduled task WinchyUsbReset (user-approved 2026-07-05) - heals the
    # wedged-CDC state without touching the cable.
    log("  running WinchyUsbReset (driver-level USB replug)...")
    subprocess.run(["schtasks", "/Run", "/TN", "WinchyUsbReset"],
                   capture_output=True, timeout=30)
    time.sleep(12)             # device re-enumerates


def enter_bootloader():
    # GENTLE pure-pyserial entry (mpremote's raw-REPL handshake fails on a
    # print-flooded console, and hammering retries WEDGES the Windows CDC -
    # learned the hard way): one clean open, Ctrl-C until a '>>>' prompt
    # appears, then type the command. Max 2 attempts, no hammering.
    for attempt in range(3):
        dev = rope_cdc_port()          # serial-number match - never the winch
        if dev is None:
            log("no rope CDC port visible (attempt %d)" % (attempt + 1))
            usb_reset()
            continue
        log("bootloader entry via %s (gentle pyserial, attempt %d)"
            % (dev, attempt + 1))
        try:
            s = serial.Serial(dev, 115200, timeout=1)
            got_prompt = False
            for _ in range(12):        # interrupt until the REPL answers
                s.write(b"\x03")
                time.sleep(0.4)
                if b">>>" in s.read(4000):
                    got_prompt = True
                    break
            log("  REPL prompt: %s" % got_prompt)
            if got_prompt:
                s.write(b"import machine\r\n")
                time.sleep(0.3)
                s.read(500)
                s.write(b"machine.bootloader()\r\n")
                time.sleep(0.5)
            s.close()
        except Exception as e:
            # A dead CDC often means the chip ALREADY left for the ROM
            # bootloader (enumeration can take ~60 s) - check that first,
            # only then try the driver-level USB reset.
            log("  serial error: %s -> waiting for ROM (up to 5 min; no"
                " usb resets - they delay the enumeration)"
                % type(e).__name__)
            dl = rope_download_port(300)
            if dl:
                return dl
            log("  no ROM port either -> WinchyUsbReset")
            usb_reset()
            continue
        dl = rope_download_port(75)   # ROM enumeration can take ~60 s
        if dl:
            return dl
        log("  ROM port did not appear")
    raise SystemExit("VERDICT: ABORT (could not enter bootloader - "
                     "likely a wedged CDC; replug the rope USB once)")



def get_announce():
    # The rope re-announces its IP on EVERY boot (announced_ip resets), and the
    # announce line carries a timestamp - a changed line during the soak means
    # the rope REBOOTED. Works on every firmware revision in the bisect.
    try:
        r = subprocess.run(["gh", "release", "view", "logs", "--repo",
                            "dspreitz/winchy-logs", "--json", "body",
                            "--jq", ".body"],
                           capture_output=True, text=True, timeout=30)
        for line in (r.stdout or "").splitlines():
            if line.startswith("rope:"):
                return line.strip()
    except Exception:
        pass
    return None


def main():
    log("=== soak round: ref=%s soak=%d min ===" % (GIT_REF, SOAK_MIN))

    # 1. checkout + clean build
    if SOAK_ONLY:
        head = GIT_REF + " (soak-only: device already flashed)"
        log(head)
    else:
        run(["git", "-C", REPO, "checkout", GIT_REF])
        head = run(["git", "-C", REPO, "log", "--oneline", "-1"]).stdout.strip()
        log("checked out: " + head)
    if SKIP_BUILD:
        log("build SKIPPED (--skip-build: using the existing bin)")
    else:
        log("building (clean, docker direct)...")
        # Call docker directly: powershell 5.1 + ErrorActionPreference=Stop
        # treats docker's stderr progress as errors when stderr is captured
        # (aborts the script), so run.ps1 is not usable from here.
        run(["docker", "run", "--rm",
             "-v", REPO + ":/repo",
             "-v", "winchy-ccache:/root/.ccache",
             "-e", "WINCHY_REPO=/repo",
             "-e", "IDF_CCACHE_ENABLE=1",
             "-e", "CLEAN=1",
             "espressif/idf:v5.5.1", "bash", "-c",
             "tr -d '\\r' < /repo/firmware/fwbuild/build.sh | bash -s -- rope"],
            timeout=1800)
        log("build done")

    # 2. software bootloader entry + flash + reset (skipped in soak-only)
    if SOAK_ONLY:
        log("flash phase SKIPPED (--soak-only)")
        return soak_and_verdict(head)
    dl = rope_download_port(5)
    if dl is None:
        log("entering bootloader via machine.bootloader() ...")
        dl = enter_bootloader()
    log("ROM download port: " + dl)
    log("flashing...")
    run([sys.executable, "-m", "esptool", "--chip", "esp32s3", "-p", dl,
         "-b", "921600", "write-flash", "-z", "0x0", BIN], timeout=300)
    log("flash done (esptool issues its post-flash reset)")

    # 3. wait for the app (CDC re-enumeration), else nudge with another reset
    t0 = time.time()
    app_up = False
    while time.time() - t0 < 90:
        if rope_cdc_port():
            app_up = True
            break
        time.sleep(3)
    if not app_up:
        log("app not up yet - issuing another ROM reset via esptool read-mac")
        dl2 = rope_download_port(10)
        if dl2:
            subprocess.run([sys.executable, "-m", "esptool", "--chip",
                            "esp32s3", "-p", dl2, "--after", "hard-reset",
                            "read-mac"], capture_output=True, timeout=60)
        t0 = time.time()
        while time.time() - t0 < 120:
            if rope_cdc_port():
                app_up = True
                break
            time.sleep(3)
    log("app enumerated: %s" % app_up)
    if not app_up:
        raise SystemExit("VERDICT: ABORT (device did not boot after flash - "
                         "needs a manual power-cycle)")

    return soak_and_verdict(head)


def soak_and_verdict(head):
    # 4. wait for WiFi (old pre-tuning builds can take >6 min to join)
    t0 = time.time()
    while time.time() - t0 < 600:
        if http_ok(ROPE_URL):
            break
        time.sleep(10)
    else:
        raise SystemExit("VERDICT: ABORT (WiFi never came up)")
    log("WiFi up after %.0f s" % (time.time() - t0))

    boots0, rst0 = fetch_rst_markers()
    log("round-start markers: boots=%s rst=%s" % (boots0, rst0))
    ann0 = get_announce()
    log("round-start announce: %r" % ann0)

    # 5. soak watch
    log("soak started (%d min, poll 30 s)" % SOAK_MIN)
    t0 = time.time()
    fails = 0
    max_consec = 0
    consec = 0
    outages = 0
    while time.time() - t0 < SOAK_MIN * 60:
        ok = http_ok(ROPE_URL)
        if ok:
            if consec >= 4:
                outages += 1
                log("outage ENDED after %d misses" % consec)
            consec = 0
        else:
            fails += 1
            consec += 1
            max_consec = max(max_consec, consec)
            if consec == 4:
                log("sustained outage detected (2 min) at t=%.0f s"
                    % (time.time() - t0))
        time.sleep(30)
    if consec >= 4:
        outages += 1

    # 6. verdict
    boots1, rst1 = fetch_rst_markers()
    log("round-end markers: boots=%s rst=%s" % (boots1, rst1))
    ann1 = get_announce()
    log("round-end announce: %r" % ann1)
    new_boots = (boots1 - boots0) if (boots0 is not None
                                      and boots1 is not None) else None
    wdt0 = sum(1 for x in rst0 if "WDT" in x)
    wdt1 = sum(1 for x in rst1 if "WDT" in x)
    # Two ropes overwrite each other's announce line in parallel operation:
    # only trust the signal when BOTH samples reference THIS board's IP.
    my_ip = ROPE_URL.split("//")[1].strip("/")
    announce_changed = (ann0 is not None and ann1 is not None
                        and my_ip in ann0 and my_ip in ann1
                        and ann0 != ann1)
    crashed = ((outages > 0)
               or (new_boots is not None and new_boots > 0)
               or (wdt1 > wdt0)                # fresh panic marker
               or announce_changed)            # rope re-announced = rebooted
    detail = {"ref": GIT_REF, "head": head, "soak_min": SOAK_MIN,
              "fails": fails, "max_consec_miss": max_consec,
              "outages": outages, "new_boots": new_boots, "wdt_delta": wdt1 - wdt0, "announce_changed": announce_changed,
              "rst_end": rst1[-3:] if rst1 else []}
    log("DETAIL: " + json.dumps(detail))
    print("VERDICT: %s" % ("CRASH" if crashed else "CLEAN"), flush=True)


main()
