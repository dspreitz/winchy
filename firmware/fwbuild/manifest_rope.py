# Winchy ROPE freeze manifest.
#
# Freezes the rope application into the firmware image. WINCHY_REPO is exported
# by build.sh and points at the repo root. The board's default frozen modules
# (asyncio, etc.) are kept via include().
#
# NOT frozen (stay on the device filesystem):
#   - boot.py / main.py  : tiny boot glue + crash guard (deployed once)
#   - secrets.py         : WiFi/GitHub/ZTP secrets (gitignored, never in a public bin)
#   - calibration.cal    : per-device force calibration
import os

R = os.environ["WINCHY_REPO"]

include("$(PORT_DIR)/boards/manifest.py")

# Drivers (AXP2101, sx126x stack, bmp280, sh1106, QMC6310, ...)
freeze(R + "/firmware/rope/lib")

# Shared modules
freeze(R + "/firmware/shared", ("protocol.py", "wifi.py", "nmea.py"))

# Rope application
freeze(R + "/firmware/rope", "config.py")
freeze(R + "/firmware/rope", "winchy")     # whole package (fusion/, sensors/, ...)
