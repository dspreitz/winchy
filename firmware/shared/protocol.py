# Winchy radio protocol: versioned binary frames shared by the rope and
# winch segments. Pure Python + struct, runs on MicroPython and CPython.
#
# All frames are little-endian and start with (version, type, seq). The
# sequence number is global per transmitter so the receiver can measure
# link loss across frame types. Receivers must silently ignore frames
# with an unknown version, type or length (decode() returns None).
#
# The link is bidirectional but asymmetric: the rope unit transmits
# TELEMETRY/TIME_SYNC/MASS/SUMMARY, and the winch unit transmits the
# low-rate LINK_REPORT back (its measured RSSI/SNR/loss of the downlink)
# whenever a TELEMETRY frame carries FLAG_REQUEST_REPORT. The feedback is
# advisory: losing it must never stall the rope->winch telemetry.

import struct

VERSION = 6

# Frame types
TELEMETRY = 1
TIME_SYNC = 2
MASS = 3
SUMMARY = 4
LINK_REPORT = 5  # winch -> rope: downlink quality as seen by the receiver
WINCH_POS = 6    # winch -> rope: surveyed (averaged) winch position, low rate

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
FLAG_REQUEST_REPORT = 0x08  # winch should reply with a LINK_REPORT
FLAG_BATTERY_LOW = 0x10     # rope battery low (set only while IDLE)
FLAG_CHARGING = 0x20        # rope battery currently charging

# WINCH_POS.status bits
WINCH_FIX = 0x01            # winch GPS currently has a position fix
WINCH_SURVEY_DONE = 0x02    # survey-in has converged to target accuracy

# version:B type:B seq:H | phase:B force:i angle:B(0.5 deg)
# altitude:H(1 m, AMSL) battery:B(0.1 V) flags:B batt_pct:B(%, 255=unknown)
# glider_speed:H(0.1 m/s) - CG-hook speed estimate
_TELEMETRY_FMT = "<BBHBiBHBBBH"
# version:B type:B seq:H | unix epoch seconds UTC:I
_TIME_SYNC_FMT = "<BBHI"
# version:B type:B seq:H | mass:H(0.1 kg) confidence:B(%)
_MASS_FMT = "<BBHHB"
# version:B type:B seq:H | duration:H(0.1 s) max force:i release alt:H(m)
# mass:H(0.1 kg)
_SUMMARY_FMT = "<BBHHiHH"
# version:B type:B seq:H | rssi:h(dBm) snr:b(dB) loss:B(%) - winch uplink
_LINK_REPORT_FMT = "<BBHhbB"
# version:B type:B seq:H | lat:i(1e-7 deg) lon:i(1e-7 deg)
# altitude:H(1 m, AMSL) hacc:B(0.1 m, saturates at 25.5) status:B
_WINCH_POS_FMT = "<BBHiiHBB"


def encode_telemetry(seq, phase, force, angle_deg, altitude_m, batt_mv,
                     flags, batt_pct=255, glider_speed_ms=0.0):
    angle = max(0, min(255, int(round(angle_deg * 2))))
    altitude = max(0, min(65535, int(round(altitude_m))))
    batt = max(0, min(255, batt_mv // 100))
    pct = batt_pct & 0xFF   # 0..100 normal; 255 = unknown / no cell (-1 -> 255)
    speed = max(0, min(65535, int(round(glider_speed_ms * 10))))
    return struct.pack(_TELEMETRY_FMT, VERSION, TELEMETRY, seq & 0xFFFF,
                       phase, force, angle, altitude, batt, flags, pct, speed)


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


def encode_link_report(seq, rssi_dbm, snr_db, loss_pct):
    rssi = max(-32768, min(32767, int(round(rssi_dbm))))
    snr = max(-128, min(127, int(round(snr_db))))
    loss = max(0, min(100, int(round(loss_pct))))
    return struct.pack(_LINK_REPORT_FMT, VERSION, LINK_REPORT, seq & 0xFFFF,
                       rssi, snr, loss)


def encode_winch_pos(seq, lat_deg, lon_deg, altitude_m, hacc_m=25.5, status=0):
    lat = max(-2147483648, min(2147483647, int(round(lat_deg * 1e7))))
    lon = max(-2147483648, min(2147483647, int(round(lon_deg * 1e7))))
    alt = max(0, min(65535, int(round(altitude_m))))
    hacc = max(0, min(255, int(round(hacc_m * 10))))
    return struct.pack(_WINCH_POS_FMT, VERSION, WINCH_POS, seq & 0xFFFF,
                       lat, lon, alt, hacc, status & 0xFF)


def decode(frame):
    """Decode any frame into a dict with a 'type' key (one of the frame
    type constants). Returns None for unknown version/type/length."""
    if frame is None or len(frame) < 4 or frame[0] != VERSION:
        return None
    ftype = frame[1]

    if ftype == TELEMETRY and len(frame) == struct.calcsize(_TELEMETRY_FMT):
        (_, _, seq, phase, force, angle, altitude, batt, flags,
         batt_pct, speed) = struct.unpack(_TELEMETRY_FMT, frame)
        return {"type": TELEMETRY, "seq": seq, "phase": phase,
                "force": force, "angle_deg": angle / 2,
                "altitude_m": altitude, "batt_v": batt / 10,
                "flags": flags, "batt_pct": batt_pct,
                "glider_speed_ms": speed / 10}

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

    if ftype == LINK_REPORT and len(frame) == struct.calcsize(_LINK_REPORT_FMT):
        _, _, seq, rssi, snr, loss = struct.unpack(_LINK_REPORT_FMT, frame)
        return {"type": LINK_REPORT, "seq": seq, "rssi_dbm": rssi,
                "snr_db": snr, "loss_pct": loss}

    if ftype == WINCH_POS and len(frame) == struct.calcsize(_WINCH_POS_FMT):
        (_, _, seq, lat, lon, alt, hacc,
         status) = struct.unpack(_WINCH_POS_FMT, frame)
        return {"type": WINCH_POS, "seq": seq, "lat": lat / 1e7,
                "lon": lon / 1e7, "altitude_m": alt, "hacc_m": hacc / 10,
                "status": status}

    return None
