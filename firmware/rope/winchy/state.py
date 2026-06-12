# Shared application state: the latest value of each measured quantity
# plus its timestamp (time.ticks_ms). Each task writes only its own fields
# and reads any others; under cooperative (asyncio) scheduling no locking
# is needed.


class State:
    def __init__(self):
        # Force (ADS1232)
        self.force_raw = 0       # last raw conversion (untared)
        self.force_offset = 0    # tare offset
        self.force_ts = 0
        self.force_errors = 0    # DRDY timeouts
        # IMU / attitude
        self.accel = (0.0, 0.0, 0.0)  # windowed mean specific force, g
        self.angle_deg = 0.0          # rope angle above ground
        self.accel_ts = 0
        # Barometer
        self.pressure_hpa = 0.0
        self.baro_ts = 0
        # GPS
        self.gps_fix = 0         # GGA fix quality (0 = none)
        self.gps_sats = 0
        self.lat = 0.0
        self.lon = 0.0
        self.alt_m = 0.0
        self.gps_ts = 0
        self.time_synced = False
        self.pending_time_sync = False  # telemetry task owes a TIME_SYNC frame
        # Power
        self.system_mv = 0
        self.batt_mv = 0
        self.batt_pct = 0
        # Radio link
        self.tx_count = 0
