# Rope segment entry point.
#
# Migration step 1 (docs/rope_segment_architecture.md): the original
# monolithic application lives unchanged in legacy.py and is started from
# here behind a crash guard, so any failure or Ctrl-C lands at the REPL
# instead of wedging the unit.

import sys

from machine import Pin

# Safe mode: hold the BOOT button (IO0) while resetting to get a bare REPL
# without starting the application.
if Pin(0, Pin.IN, Pin.PULL_UP).value() == 0:
    print("SAFE MODE: BOOT button held, application not started")
else:
    try:
        import legacy  # original application; runs forever
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
