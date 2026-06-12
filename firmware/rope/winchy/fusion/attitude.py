# Attitude math. Pure functions, no hardware imports - runs and tests on
# desktop CPython as well as on the device.

import math


def rope_inclination(ax, ay, az):
    """Angle between the rope (device Y axis) and gravity, in degrees.

    Immune to the rope spinning around its own axis. Input is specific
    force in g. 0 = rope vertical, 90 = rope horizontal.

    Only valid while kinematic acceleration is small compared to gravity;
    during ground roll / rotation the Kalman filter must take over (see
    docs/winch_launch_physics.md).
    """
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    angle_rad = math.acos(abs(ay / norm))
    return math.degrees(angle_rad)


def rope_angle_above_ground(ax, ay, az):
    """Rope elevation angle relative to the ground ("Seilwinkel")."""
    return 90 - rope_inclination(ax, ay, az)
