"""
Utility functions for coordinate conversions and quaternions.

Responsibilities:
- ENU (East-North-Up) to WGS-84 (GPS) coordinate conversion
- Euler angles to quaternion conversion
- Other math utilities
"""

import numpy as np
from typing import Tuple


# Reference point for ENU→WGS84 conversion (arbitrary location)
# Using a location in Colorado as reference
REF_LAT = 39.7392  # degrees
REF_LON = -104.9903  # degrees
REF_ALT = 1655.0  # meters

# Earth radius (WGS-84 semi-major axis)
EARTH_RADIUS = 6378137.0  # meters


def enu_to_wgs84(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """
    Convert ENU (East-North-Up) coordinates to WGS-84 (lat, lon, alt).

    Uses a flat-Earth approximation (tangent plane) suitable for small areas.

    Args:
        x: East coordinate in meters
        y: North coordinate in meters
        z: Up coordinate in meters (height above reference)

    Returns:
        (latitude, longitude, altitude) in degrees and meters
    """
    # Convert meters to degrees (approximate)
    meters_per_degree_lat = 111132.92  # At equator, varies slightly with latitude
    meters_per_degree_lon = 111132.92 * np.cos(np.radians(REF_LAT))

    lat = REF_LAT + (y / meters_per_degree_lat)
    lon = REF_LON + (x / meters_per_degree_lon)
    alt = REF_ALT + z

    return (lat, lon, alt)


def wgs84_to_enu(lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
    """
    Convert WGS-84 (lat, lon, alt) to ENU (East-North-Up) coordinates.

    Inverse of enu_to_wgs84.

    Args:
        lat: Latitude in degrees
        lon: Longitude in degrees
        alt: Altitude in meters

    Returns:
        (x, y, z) in ENU frame (meters)
    """
    meters_per_degree_lat = 111132.92
    meters_per_degree_lon = 111132.92 * np.cos(np.radians(REF_LAT))

    y = (lat - REF_LAT) * meters_per_degree_lat
    x = (lon - REF_LON) * meters_per_degree_lon
    z = alt - REF_ALT

    return (x, y, z)


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion.

    Args:
        roll: Roll angle in radians (rotation around x-axis)
        pitch: Pitch angle in radians (rotation around y-axis)
        yaw: Yaw angle in radians (rotation around z-axis)

    Returns:
        (x, y, z, w) quaternion components
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (x, y, z, w)


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).

    Args:
        x, y, z, w: Quaternion components

    Returns:
        (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi] range.

    Args:
        angle: Angle in radians

    Returns:
        Normalized angle in [-pi, pi]
    """
    return np.arctan2(np.sin(angle), np.cos(angle))
