# Winchy - glider winch rope force & advice system
# Copyright (C) 2026 Dominic Spreitz
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version. Distributed WITHOUT ANY WARRANTY.
# See the GNU General Public License for more details, and the LICENSE
# file or <https://www.gnu.org/licenses/> for the full text.
#
# SPDX-License-Identifier: GPL-3.0-or-later

# Rope segment entry point.
#
# The application (winchy/app.py, asyncio runtime) is started behind a
# crash guard. A deliberate Ctrl-C drops to the REPL for debugging; an
# unexpected crash self-heals by resetting after an interruptible countdown,
# so a transient boot failure (e.g. an ADS1232 DRDY timeout at tare) can't
# leave a fielded unit dead at the REPL. Hold the BOOT button during reset
# for a bare REPL (or Ctrl-C during the countdown).

import sys
import time

import machine
import micropython
from machine import Pin

CRASH_RESET_DELAY_S = 10   # interruptible window before auto-reset on a crash

# Safe mode: hold the BOOT button (IO0) while resetting to get a bare REPL
# without starting the application.
if Pin(0, Pin.IN, Pin.PULL_UP).value() == 0:
    print("SAFE MODE: BOOT button held, application not started")
else:
    # Ignore Ctrl-C during the fragile startup window. A host attaching to the
    # USB-CDC during boot (mpremote sends Ctrl-C on every connect) otherwise
    # raised KeyboardInterrupt here and dropped to the REPL ("won't start"
    # gremlin). _main re-enables it once the app is up, so a deliberate Ctrl-C
    # still interrupts the running app for deploys. BOOT-button safe mode above
    # is the escape hatch if startup itself misbehaves.
    micropython.kbd_intr(-1)
    try:
        from winchy import app
        app.run()  # runs forever (re-enables Ctrl-C early in _main)
    except KeyboardInterrupt:
        micropython.kbd_intr(3)
        print("Application interrupted, dropping to REPL")
    except Exception as e:
        micropython.kbd_intr(3)   # keep the countdown below interruptible
        sys.print_exception(e)
        try:
            with open("crash.log", "w") as f:
                sys.print_exception(e, f)
        except OSError:
            pass
        # Self-heal in the field; stay interruptible for the bench.
        try:
            print("Application crashed (traceback in crash.log). Resetting in "
                  "%ds - Ctrl-C for REPL." % CRASH_RESET_DELAY_S)
            for _ in range(CRASH_RESET_DELAY_S):
                time.sleep(1)
            machine.reset()
        except KeyboardInterrupt:
            print("Auto-reset cancelled, dropping to REPL")
