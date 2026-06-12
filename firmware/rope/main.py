# Rope segment entry point.
#
# The application (winchy/app.py, asyncio runtime) is started behind a
# crash guard, so any failure or Ctrl-C lands at the REPL instead of
# wedging the unit. Hold the BOOT button during reset for a bare REPL.

import sys

from machine import Pin

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
        print("Application crashed (traceback in crash.log), dropping to REPL")
