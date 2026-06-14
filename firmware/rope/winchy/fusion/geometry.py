# Winch-relative launch geometry. Pure functions, no hardware imports - runs
# and tests on desktop CPython as well as on the device.
#
# Both units carry a GPS, so differencing the two positions gives the
# winch->rope baseline. Because both receivers share the same constellation,
# the common-mode errors (ionosphere, ephemeris) largely cancel in the
# difference, so the relative geometry is tighter than either absolute fix.
# All distances are metres, angles degrees.
#
# Coordinates use a local-tangent (equirectangular) plane centred between the
# two points. At the ~km baselines of a winch launch this is exact to the
# millimetre, and avoids the cost/instability of full great-circle formulae on
# the device.

import math

_EARTH_R = 6371000.0  # mean Earth radius, m


def enu_offset_m(lat_ref, lon_ref, lat, lon):
    """East,North offset (m) of (lat,lon) from (lat_ref,lon_ref) on the local
    tangent plane."""
    mlat = math.radians((lat_ref + lat) / 2)
    east = math.radians(lon - lon_ref) * math.cos(mlat) * _EARTH_R
    north = math.radians(lat - lat_ref) * _EARTH_R
    return east, north


def horizontal_distance_m(lat1, lon1, lat2, lon2):
    """Ground distance between two lat/lon points, metres."""
    east, north = enu_offset_m(lat1, lon1, lat2, lon2)
    return math.sqrt(east * east + north * north)


def bearing_deg(lat1, lon1, lat2, lon2):
    """Initial bearing from point 1 to point 2, degrees clockwise from north
    (0..360). At launch start this is the ground-roll heading - the direction
    from the winch towards the glider."""
    east, north = enu_offset_m(lat1, lon1, lat2, lon2)
    return math.degrees(math.atan2(east, north)) % 360.0


def elevation_angle_deg(horizontal_m, dh_m):
    """Elevation angle (deg) of a point dh_m above and horizontal_m away."""
    return math.degrees(math.atan2(dh_m, horizontal_m))


def slant_distance_m(horizontal_m, dh_m):
    """Straight-line distance from a horizontal separation and height delta."""
    return math.sqrt(horizontal_m * horizontal_m + dh_m * dh_m)


def winch_relative(winch_lat, winch_lon, winch_alt_m,
                   rope_lat, rope_lon, rope_alt_m, hook_distance_m=5.0):
    """Bundle the winch-relative geometry from the two GPS fixes + altitudes.

    Returns a dict:
      horizontal_m    ground distance winch -> rope segment
      dh_m            rope segment height above the winch
      slant_m         straight-line winch -> rope distance
      elevation_deg   elevation angle of the rope segment seen from the winch
      bearing_deg     bearing winch -> rope (ground-roll heading at launch)
      cable_length_m  winch -> glider hook ~ slant + hook_distance_m

    Sampled at force-onset (cable taut, both ends on the ground) dh_m ~ 0, so
    horizontal_m ~ slant_m and cable_length_m is the initial cable length.
    Through the climb it is the live winch->glider distance, whose rate is the
    reel-in speed and whose elevation_deg is the launch profile.
    """
    horizontal = horizontal_distance_m(winch_lat, winch_lon, rope_lat, rope_lon)
    dh = rope_alt_m - winch_alt_m
    slant = slant_distance_m(horizontal, dh)
    return {
        "horizontal_m": horizontal,
        "dh_m": dh,
        "slant_m": slant,
        "elevation_deg": elevation_angle_deg(horizontal, dh),
        "bearing_deg": bearing_deg(winch_lat, winch_lon, rope_lat, rope_lon),
        "cable_length_m": slant + hook_distance_m,
    }
