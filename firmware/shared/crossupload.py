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

# Cross-upload decision logic, shared by the rope and the winch and extracted
# so it is pure and host-testable. A WebGUI "Upload log" click on one unit asks
# the OTHER unit (over the radio) to upload too: UPLOAD_CMD is retried until
# UPLOAD_ACK confirms it; an incoming UPLOAD_CMD triggers one upload per nonce
# (retries of the same nonce are deduplicated). The callers own the radio, the
# clocks (they pass elapsed milliseconds) and the state fields.

CMD_RETRY_MS = 1000        # gap between UPLOAD_CMD retries
CMD_DEDUP_EXPIRY_MS = 30000  # forget the last-seen nonce after this: a peer
                             # that REBOOTED restarts its nonce counter, so a
                             # reused nonce must trigger again once the retry
                             # burst (~5 s) is long over - without this, the
                             # first click after a peer reboot was silently
                             # swallowed (ACKed but no upload).


def tx_plan(ack_nonce, cmd_nonce, cmd_tries, ms_since_last_cmd,
            retry_ms=CMD_RETRY_MS):
    """Decide the single cross-upload frame to TX this cycle (half-duplex:
    one frame per opportunity). Priority: ACK a received UPLOAD_CMD first,
    else (re)send our own UPLOAD_CMD while tries remain, spaced retry_ms.

    Returns (kind, nonce, new_tries, cmd_done):
      kind     : "ack" | "cmd" | None (nothing to send)
      nonce    : the nonce to put in the frame (None when kind is None)
      new_tries: remaining UPLOAD_CMD tries after this cycle
      cmd_done : True when this was the final CMD send (caller clears the nonce)
    """
    if ack_nonce is not None:
        return "ack", ack_nonce, cmd_tries, False
    if (cmd_nonce is not None and cmd_tries > 0
            and ms_since_last_cmd >= retry_ms):
        tries = cmd_tries - 1
        return "cmd", cmd_nonce, tries, tries <= 0
    return None, None, cmd_tries, False


def accept_cmd(nonce, last_nonce, ms_since_last,
               expiry_ms=CMD_DEDUP_EXPIRY_MS):
    """Should an incoming UPLOAD_CMD trigger an upload? Dedups the sender's
    retry burst (same nonce within expiry_ms), but a stale latch expires so a
    rebooted peer's reused nonce still triggers. The ACK is sent regardless -
    this only gates starting the upload."""
    return (last_nonce is None or nonce != last_nonce
            or ms_since_last > expiry_ms)
