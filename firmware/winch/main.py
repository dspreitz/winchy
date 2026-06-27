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

# Winch entry point.
#
# The whole application lives in winch_app.py, which is FROZEN into the
# firmware (see firmware/fwbuild/manifest_winch.py). This launcher is the only
# thing on the filesystem (besides secrets.py): importing winch_app runs its
# top-level crash-guard + asyncio app, which never returns. Keeping the launcher
# tiny means the application is shipped inside the .bin and a normal flash needs
# no mpremote deploy of the app code.
import winch_app  # noqa: F401  (runs the app on import)
