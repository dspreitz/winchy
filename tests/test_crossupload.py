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

# Host tests for the cross-upload decision logic (firmware/shared/
# crossupload.py): the TX plan (ACK priority, CMD retry pacing, give-up) and
# the incoming-CMD dedup with expiry (a rebooted peer restarts its nonce
# counter, so a stale dedup latch used to swallow the first click after a peer
# reboot - ACKed but no upload).

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware",
                                "shared"))

from crossupload import tx_plan, accept_cmd


# --- tx_plan: what to send this cycle ---------------------------------------

def test_nothing_pending_sends_nothing():
    assert tx_plan(None, None, 0, 99999) == (None, None, 0, False)


def test_ack_has_priority_over_cmd():
    kind, nonce, tries, done = tx_plan(7, 3, 5, 99999)
    assert kind == "ack" and nonce == 7
    assert tries == 5 and done is False          # the CMD state is untouched


def test_cmd_sent_when_due_and_counts_down():
    kind, nonce, tries, done = tx_plan(None, 3, 5, 1000)
    assert kind == "cmd" and nonce == 3
    assert tries == 4 and done is False


def test_cmd_respects_retry_spacing():
    # Only 500 ms since the last send -> hold this cycle (retries are ~1 s apart)
    assert tx_plan(None, 3, 5, 500) == (None, None, 5, False)


def test_final_cmd_try_reports_done():
    kind, nonce, tries, done = tx_plan(None, 3, 1, 5000)
    assert kind == "cmd" and nonce == 3
    assert tries == 0 and done is True           # caller clears the nonce


def test_no_cmd_after_tries_exhausted():
    assert tx_plan(None, 3, 0, 99999) == (None, None, 0, False)


def test_first_cmd_send_is_immediate():
    # cross_cmd_ts starts at 0, so the elapsed time is huge -> first send now.
    kind, _, _, _ = tx_plan(None, 1, 5, 10 ** 9)
    assert kind == "cmd"


def test_cmd_due_when_elapsed_wrapped_negative():
    # MicroPython ticks_diff wraps at 2^30 ms (~12.4 days) into [-2^29, 2^29):
    # after ~6.2 days of uptime the handlers' "0 = send now" sentinel yields a
    # NEGATIVE elapsed. Negative can only mean "ancient" -> the send is DUE;
    # without this the upload click silently sent no radio CMD for days.
    kind, nonce, tries, done = tx_plan(None, 1, 5, -(2 ** 28))
    assert kind == "cmd" and nonce == 1 and tries == 4 and done is False


# --- accept_cmd: dedup of the sender's retry burst, with expiry --------------

def test_first_cmd_ever_triggers():
    assert accept_cmd(1, None, 0) is True


def test_retry_burst_same_nonce_deduplicated():
    # The sender retries the same nonce ~1 s apart until ACKed; only the first
    # copy may start an upload.
    assert accept_cmd(1, 1, 1000) is False
    assert accept_cmd(1, 1, 5000) is False


def test_new_nonce_triggers():
    assert accept_cmd(2, 1, 1000) is True


def test_reused_nonce_triggers_after_expiry():
    # THE bug this guards: peer rebooted, its counter restarted, the reused
    # nonce arrived minutes later - must trigger again, not be swallowed.
    assert accept_cmd(1, 1, 31000) is True
    assert accept_cmd(1, 1, 30000) is False      # still within the window


def test_reused_nonce_triggers_when_elapsed_wrapped_negative():
    # >6.2 days since the last CMD wraps ticks_diff negative - that is ancient,
    # so the dedup latch must count as expired, not as "just seen".
    assert accept_cmd(1, 1, -(2 ** 28)) is True
