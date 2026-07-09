# Winchy WINCH freeze manifest.
#
# Freezes the winch application into the firmware image. WINCHY_REPO is exported
# by build.sh and points at the repo root.
#
# NOT frozen (stay on the device filesystem):
#   - main.py     : 2-line launcher (`import winch_app`)
#   - secrets.py  : WiFi/GitHub secrets (gitignored, never in a public bin)
import os

R = os.environ["WINCHY_REPO"]

include("$(PORT_DIR)/boards/manifest.py")

# Radio driver (official micropython-lib lora package, shared under rope/lib)
# + OLED driver
freeze(R + "/firmware/rope/lib", ("lora/__init__.py", "lora/modem.py",
                                  "lora/sx126x.py", "lora/sync_modem.py",
                                  "lora/async_modem.py"))
freeze(R + "/firmware/winch", "ssd1306.py")

# Shared modules
freeze(R + "/firmware/shared", ("protocol.py", "nmea.py", "wifi.py", "survey.py",
                                "gpstime.py", "crossupload.py", "eventlog.py"))

# Winch application (the former monolith main.py)
freeze(R + "/firmware/winch", "winch_app.py")
