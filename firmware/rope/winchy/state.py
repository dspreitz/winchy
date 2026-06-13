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
        self.accel = (0.0, 0.0, 0.0)     # instantaneous specific force, g
        self.gyro_dps = (0.0, 0.0, 0.0)  # bias-corrected angular rate
        self.mag = (0.0, 0.0, 0.0)       # magnetometer field, uT (x, y, z)
        self.angle_deg = 0.0             # rope angle above ground (Kalman)
        self.accel_ts = 0
        # Tow phase (protocol.PHASE_*; the state machine will drive this)
        self.phase = 0  # PHASE_IDLE
        # Barometer
        self.pressure_hpa = 0.0
        self.baro_ts = 0
        self.qnh_hpa = 0.0      # sea-level reference, GPS-calibrated; 0 = none
        self.baro_alt_m = 0.0   # altitude from pressure + qnh_hpa
        self.climb_rate_ms = 0.0  # vertical Kalman estimate
        # GPS
        self.gps_fix = 0         # GGA fix quality (0 = none)
        self.gps_sats = 0
        self.lat = 0.0
        self.lon = 0.0
        self.alt_m = 0.0
        self.gps_ts = 0
        self.time_synced = False
        self.pending_time_sync = False  # telemetry task owes a TIME_SYNC frame
        self.log_start = None    # "yyyymmdd-hhmm" session start (first GPS time)
        # Power
        self.system_mv = 0
        self.batt_mv = 0
        self.batt_pct = 0        # AXP2101 fuel gauge %, -1 if no cell
        self.batt_low = False    # rope battery low (checked while IDLE)
        self.charging = False    # AXP2101 reports the cell is charging
        # Radio link
        self.tx_count = 0
        # Downlink quality as reported by the winch (LINK_REPORT); for ADR.
        self.link_rssi_dbm = 0
        self.link_snr_db = 0
        self.link_loss_pct = 0
        self.link_report_ts = 0  # time.ticks_ms of last report; 0 = never
        self.tx_power_dbm = 0    # current radio TX power, driven by ADR
