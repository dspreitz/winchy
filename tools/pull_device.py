"""Pull all files from a MicroPython device that is busy running main.py.

Waits for the device to be re-plugged (port disappears, then reappears),
opens the port immediately and spams Ctrl-C so the interrupt lands during
early startup. Then enters raw REPL, walks the device filesystem and copies
every file to a local directory.

Usage: python tools/pull_device.py <dest_dir>
"""

import base64
import os
import sys
import time

import serial
from serial.tools import list_ports

DEST = sys.argv[1] if len(sys.argv) > 1 else "device_backup"
ESPRESSIF_VID = 0x303A
REPLUG_TIMEOUT = 600  # seconds to wait for the user to replug


def find_port():
    for p in list_ports.comports():
        if p.vid == ESPRESSIF_VID:
            return p.device
    return None


def wait_for_replug():
    """Wait until the device port vanishes and comes back; return port name."""
    print("Waiting for device to be unplugged...", flush=True)
    deadline = time.time() + REPLUG_TIMEOUT
    while find_port() is not None:
        if time.time() > deadline:
            raise SystemExit("Timed out waiting for unplug")
        time.sleep(0.2)
    print("Device unplugged. Waiting for it to come back...", flush=True)
    while True:
        port = find_port()
        if port:
            print("Device reappeared on %s" % port, flush=True)
            return port
        if time.time() > deadline:
            raise SystemExit("Timed out waiting for replug")
        time.sleep(0.1)


def open_port(port):
    # Retry: Windows needs a moment after enumeration before open succeeds.
    for _ in range(50):
        try:
            return serial.Serial(port, 115200, timeout=1)
        except serial.SerialException:
            time.sleep(0.1)
    raise SystemExit("Could not open %s after replug" % port)


def drain(s, wait=0.3):
    time.sleep(wait)
    data = b""
    while s.in_waiting:
        data += s.read(s.in_waiting)
        time.sleep(0.05)
    return data


def interrupt(s, duration=45):
    """Spam Ctrl-C until the REPL prompt appears."""
    deadline = time.time() + duration
    seen = b""
    while time.time() < deadline:
        try:
            s.write(b"\x03")
            out = drain(s, 0.05)
        except serial.SerialException:
            return None  # port dropped (device rebooted) - caller retries
        if out:
            sys.stdout.write(out.decode("utf-8", "replace"))
            sys.stdout.flush()
        seen = (seen + out)[-2000:]
        if b">>>" in seen:
            time.sleep(0.5)
            s.reset_input_buffer()
            s.write(b"\r\n")
            out = drain(s, 0.5)
            if b">>>" in out:
                return True
            seen = b""
    return False


def enter_raw_repl(s):
    s.write(b"\x01")
    out = drain(s, 0.5)
    return b"raw REPL" in out


def exec_raw(s, code, timeout=20):
    """Execute code in raw REPL, return (stdout, stderr)."""
    s.reset_input_buffer()
    s.write(code.encode() + b"\x04")
    deadline = time.time() + timeout
    buf = b""
    while b"OK" not in buf:
        if time.time() > deadline:
            raise RuntimeError("no OK after exec: %r" % buf[:200])
        buf += s.read(1)
    cur = b""
    parts = []
    while len(parts) < 2:
        if time.time() > deadline:
            raise RuntimeError("timeout reading exec output")
        c = s.read(1)
        if not c:
            continue
        if c == b"\x04":
            parts.append(cur)
            cur = b""
        else:
            cur += c
    return parts[0], parts[1]


def pull_all(s):
    out, err = exec_raw(
        s,
        "import os\n"
        "def walk(d):\n"
        "    for e in os.ilistdir(d):\n"
        "        p = (d + '/' + e[0]) if d != '/' else '/' + e[0]\n"
        "        if e[1] & 0x4000:\n"
        "            print('D', p)\n"
        "            walk(p)\n"
        "        else:\n"
        "            print('F', e[3] if len(e) > 3 else 0, p)\n"
        "walk('/')\n",
    )
    if err:
        raise SystemExit("listing error: %s" % err.decode())
    listing = out.decode().strip().splitlines()
    print("Device filesystem:")
    for line in listing:
        print("  " + line)

    files = []
    for line in listing:
        parts = line.split(" ", 2)
        if parts[0] == "F":
            files.append((parts[2], int(parts[1])))

    exec_raw(s, "import ubinascii\n")
    for path, size in files:
        local = os.path.join(DEST, path.lstrip("/").replace("/", os.sep))
        os.makedirs(os.path.dirname(local) or ".", exist_ok=True)
        print("pulling %s (%d bytes) -> %s" % (path, size, local), flush=True)
        out, err = exec_raw(
            s,
            "f = open(%r, 'rb')\n"
            "while True:\n"
            "    b = f.read(256)\n"
            "    if not b: break\n"
            "    print(ubinascii.b2a_base64(b).decode().strip())\n"
            "f.close()\n" % path,
            timeout=120,
        )
        if err:
            print("  ERROR reading %s: %s" % (path, err.decode()))
            continue
        data = b"".join(base64.b64decode(l) for l in out.decode().split())
        with open(local, "wb") as fh:
            fh.write(data)
        if len(data) != size:
            print("  WARNING: size mismatch (got %d, expected %d)" % (len(data), size))
    return len(files)


def main():
    port = wait_for_replug()
    while True:
        s = open_port(port)
        print("Port open, sending Ctrl-C...", flush=True)
        got = interrupt(s)
        if got:
            break
        try:
            s.close()
        except serial.SerialException:
            pass
        if got is None:
            print("Port dropped (device rebooted), re-attaching...", flush=True)
            time.sleep(0.5)
            port = find_port() or port
            continue
        raise SystemExit("Could not interrupt the program")

    print("\nGot REPL prompt. Entering raw REPL...")
    if not enter_raw_repl(s):
        raise SystemExit("Could not enter raw REPL")
    n = pull_all(s)
    s.write(b"\x02")  # back to friendly REPL; program stays stopped
    drain(s)
    s.close()
    print("Done. %d files pulled to %s" % (n, DEST))


if __name__ == "__main__":
    main()
