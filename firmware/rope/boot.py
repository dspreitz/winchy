# boot.py — intentionally minimal.
#
# boot.py runs before the USB console is reliably serviceable, so nothing
# may run here that can block or crash. All application startup lives in
# main.py, where a wedged program can still be interrupted and diagnosed.
# See docs/rope_segment_architecture.md.
