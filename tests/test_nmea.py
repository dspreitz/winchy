import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware",
                                "shared"))

import nmea


def test_gga_with_fix():
    m = nmea.parse_nmea(
        "$GPGGA,141744.00,4810.1234,N,01142.5678,E,1,08,0.9,520.3,M,47,M,,*00")
    assert m["type"] == "GGA"
    assert m["fix"] == 1
    assert m["sats"] == 8
    assert abs(m["lat"] - (48 + 10.1234 / 60)) < 1e-6
    assert abs(m["lon"] - (11 + 42.5678 / 60)) < 1e-6
    assert abs(m["alt_m"] - 520.3) < 1e-6


def test_gga_no_fix_blank_fields():
    # a real no-fix sentence captured from the bench u-blox 7
    m = nmea.parse_nmea(b"$GPGGA,141744.00,,,,,0,00,99.99,,,,,,*65")
    assert m["type"] == "GGA"
    assert m["fix"] == 0
    assert m["sats"] == 0
    assert m["lat"] is None and m["lon"] is None and m["alt_m"] is None


def test_rmc_void_and_valid():
    void = nmea.parse_nmea("$GPRMC,141744.00,V,,,,,,,140626,,,N*79")
    assert void["type"] == "RMC"
    assert void["valid"] is False
    assert void["speed_ms"] is None
    ok = nmea.parse_nmea(
        "$GPRMC,141744.00,A,4810.0,N,01142.0,E,10.0,90.0,140626,,,A*00")
    assert ok["valid"] is True
    assert abs(ok["speed_ms"] - 10.0 * 0.514444) < 1e-3
    assert ok["datetime"] == (2026, 6, 14, 14, 17, 44)


def test_gn_talker_and_rejects():
    assert nmea.parse_nmea("$GPGSV,6,1,22,02,,,22*78") is None  # unused sentence
    assert nmea.parse_nmea("garbage") is None
    assert nmea.parse_nmea(b"") is None
    gn = nmea.parse_nmea(
        "$GNGGA,1,4810.0,N,01142.0,E,1,08,0.9,500,M,47,M,,*00")  # $GN talker
    assert gn["type"] == "GGA" and gn["fix"] == 1
