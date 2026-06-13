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
from machine import Pin

CRASH_RESET_DELAY_S = 10   # interruptible window before auto-reset on a crash

# Safe mode: hold the BOOT button (IO0) while resetting to get a bare REPL
# without starting the application.
if Pin(0, Pin.IN, Pin.PULL_UP).value() == 0:
    print("SAFE MODE: BOOT button held, application not started")
else:
    try:
        from winchy import app
        app.run()  # runs forever
    except KeyboardInterrupt:
        print("Application interrupted, dropping to REPL")
    except Exception as e:
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
