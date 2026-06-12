# Winchy radio protocol: versioned binary frames shared by the rope and
# winch segments. Pure Python + struct, runs on MicroPython and CPython.
#
# All frames are little-endian and start with (version, type, seq). The
# sequence number is global per transmitter so the receiver can measure
# link loss across frame types. Receivers must silently ignore frames
# with an unknown version, type or length (decode() returns None).

import struct

VERSION = 1

# Frame types
TELEMETRY = 1
TIME_SYNC = 2
MASS = 3
SUMMARY = 4

# Tow phases (TELEMETRY.phase) - see docs/winch_launch_physics.md
PHASE_IDLE = 0
PHASE_SLACK_OUT = 1
PHASE_GROUND_ROLL = 2
PHASE_ROTATION = 3
PHASE_CLIMB = 4
PHASE_TOP = 5
PHASE_RELEASED = 6
PHASE_LINK_BREAK = 7

PHASE_NAMES = {
    PHASE_IDLE: "IDLE",
    PHASE_SLACK_OUT: "SLACK",
    PHASE_GROUND_ROLL: "ROLL",
    PHASE_ROTATION: "ROTATE",
    PHASE_CLIMB: "CLIMB",
    PHASE_TOP: "TOP",
    PHASE_RELEASED: "RELEASED",
    PHASE_LINK_BREAK: "BREAK!",
}

# TELEMETRY.flags bits
FLAG_FORCE_UNCALIBRATED = 0x01  # force field is raw ADC counts, not newtons
FLAG_GPS_FIX = 0x02
FLAG_TIME_SYNCED = 0x04

# version:B type:B seq:H | phase:B force:i angle:B(0.5 deg) pressure:H(0.1
# hPa) battery:B(0.1 V) flags:B
_TELEMETRY_FMT = "<BBHBiBHBB"
# version:B type:B seq:H | unix epoch seconds UTC:I
_TIME_SYNC_FMT = "<BBHI"
# version:B type:B seq:H | mass:H(0.1 kg) confidence:B(%)
_MASS_FMT = "<BBHHB"
# version:B type:B seq:H | duration:H(0.1 s) max force:i release alt:H(m)
# mass:H(0.1 kg)
_SUMMARY_FMT = "<BBHHiHH"


def encode_telemetry(seq, phase, force, angle_deg, pressure_hpa, batt_mv,
                     flags):
    angle = max(0, min(255, int(round(angle_deg * 2))))
    pressure = max(0, min(65535, int(round(pressure_hpa * 10))))
    batt = max(0, min(255, batt_mv // 100))
    return struct.pack(_TELEMETRY_FMT, VERSION, TELEMETRY, seq & 0xFFFF,
                       phase, force, angle, pressure, batt, flags)


def encode_time_sync(seq, epoch_s):
    return struct.pack(_TIME_SYNC_FMT, VERSION, TIME_SYNC, seq & 0xFFFF,
                       epoch_s)


def encode_mass(seq, mass_kg, confidence_pct):
    mass = max(0, min(65535, int(round(mass_kg * 10))))
    return struct.pack(_MASS_FMT, VERSION, MASS, seq & 0xFFFF, mass,
                       min(100, confidence_pct))


def encode_summary(seq, duration_s, max_force, release_alt_m, mass_kg):
    duration = max(0, min(65535, int(round(duration_s * 10))))
    alt = max(0, min(65535, int(round(release_alt_m))))
    mass = max(0, min(65535, int(round(mass_kg * 10))))
    return struct.pack(_SUMMARY_FMT, VERSION, SUMMARY, seq & 0xFFFF,
                       duration, max_force, alt, mass)


def decode(frame):
    """Decode any frame into a dict with a 'type' key (one of the frame
    type constants). Returns None for unknown version/type/length."""
    if frame is None or len(frame) < 4 or frame[0] != VERSION:
        return None
    ftype = frame[1]

    if ftype == TELEMETRY and len(frame) == struct.calcsize(_TELEMETRY_FMT):
        (_, _, seq, phase, force, angle, pressure, batt,
         flags) = struct.unpack(_TELEMETRY_FMT, frame)
        return {"type": TELEMETRY, "seq": seq, "phase": phase,
                "force": force, "angle_deg": angle / 2,
                "pressure_hpa": pressure / 10, "batt_v": batt / 10,
                "flags": flags}

    if ftype == TIME_SYNC and len(frame) == struct.calcsize(_TIME_SYNC_FMT):
        _, _, seq, epoch_s = struct.unpack(_TIME_SYNC_FMT, frame)
        return {"type": TIME_SYNC, "seq": seq, "epoch_s": epoch_s}

    if ftype == MASS and len(frame) == struct.calcsize(_MASS_FMT):
        _, _, seq, mass, confidence = struct.unpack(_MASS_FMT, frame)
        return {"type": MASS, "seq": seq, "mass_kg": mass / 10,
                "confidence_pct": confidence}

    if ftype == SUMMARY and len(frame) == struct.calcsize(_SUMMARY_FMT):
        (_, _, seq, duration, max_force, alt,
         mass) = struct.unpack(_SUMMARY_FMT, frame)
        return {"type": SUMMARY, "seq": seq, "duration_s": duration / 10,
                "max_force": max_force, "release_alt_m": alt,
                "mass_kg": mass / 10}

    return None
