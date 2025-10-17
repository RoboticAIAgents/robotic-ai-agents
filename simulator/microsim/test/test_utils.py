"""Unit tests for utility functions."""

import pytest
import numpy as np
from microsim.utils import (
    enu_to_wgs84,
    wgs84_to_enu,
    euler_to_quaternion,
    quaternion_to_euler,
    normalize_angle,
    REF_LAT,
    REF_LON,
    REF_ALT
)


@pytest.mark.unit
class TestCoordinateConversion:
    """Test ENU ↔ WGS84 coordinate conversions."""

    def test_enu_origin_to_wgs84(self):
        """Test that ENU origin maps to reference point."""
        lat, lon, alt = enu_to_wgs84(0.0, 0.0, 0.0)

        assert abs(lat - REF_LAT) < 1e-6
        assert abs(lon - REF_LON) < 1e-6
        assert abs(alt - REF_ALT) < 1e-6

    def test_wgs84_reference_to_enu(self):
        """Test that reference point maps to ENU origin."""
        x, y, z = wgs84_to_enu(REF_LAT, REF_LON, REF_ALT)

        assert abs(x) < 1e-6
        assert abs(y) < 1e-6
        assert abs(z) < 1e-6

    def test_enu_to_wgs84_roundtrip(self):
        """Test that ENU→WGS84→ENU preserves coordinates."""
        x_orig, y_orig, z_orig = 100.0, 200.0, 50.0

        lat, lon, alt = enu_to_wgs84(x_orig, y_orig, z_orig)
        x_back, y_back, z_back = wgs84_to_enu(lat, lon, alt)

        assert abs(x_back - x_orig) < 0.01  # Within 1cm
        assert abs(y_back - y_orig) < 0.01
        assert abs(z_back - z_orig) < 0.01

    def test_wgs84_to_enu_roundtrip(self):
        """Test that WGS84→ENU→WGS84 preserves coordinates."""
        lat_orig = REF_LAT + 0.001  # ~111 meters north
        lon_orig = REF_LON + 0.001  # ~85 meters east
        alt_orig = REF_ALT + 25.0

        x, y, z = wgs84_to_enu(lat_orig, lon_orig, alt_orig)
        lat_back, lon_back, alt_back = enu_to_wgs84(x, y, z)

        assert abs(lat_back - lat_orig) < 1e-9
        assert abs(lon_back - lon_orig) < 1e-9
        assert abs(alt_back - alt_orig) < 1e-6

    def test_enu_east_increases_longitude(self):
        """Test that moving east increases longitude."""
        lat1, lon1, _ = enu_to_wgs84(0.0, 0.0, 0.0)
        lat2, lon2, _ = enu_to_wgs84(100.0, 0.0, 0.0)  # 100m east

        assert lat2 == lat1  # Latitude unchanged
        assert lon2 > lon1  # Longitude increased

    def test_enu_north_increases_latitude(self):
        """Test that moving north increases latitude."""
        lat1, lon1, _ = enu_to_wgs84(0.0, 0.0, 0.0)
        lat2, lon2, _ = enu_to_wgs84(0.0, 100.0, 0.0)  # 100m north

        assert lat2 > lat1  # Latitude increased
        assert lon2 == lon1  # Longitude unchanged

    def test_enu_up_increases_altitude(self):
        """Test that moving up increases altitude."""
        _, _, alt1 = enu_to_wgs84(0.0, 0.0, 0.0)
        _, _, alt2 = enu_to_wgs84(0.0, 0.0, 100.0)  # 100m up

        assert alt2 > alt1
        assert abs(alt2 - alt1 - 100.0) < 1e-6


@pytest.mark.unit
class TestQuaternionConversion:
    """Test Euler ↔ Quaternion conversions."""

    def test_zero_rotation(self):
        """Test that zero rotation produces identity quaternion."""
        x, y, z, w = euler_to_quaternion(0.0, 0.0, 0.0)

        assert abs(x) < 1e-9
        assert abs(y) < 1e-9
        assert abs(z) < 1e-9
        assert abs(w - 1.0) < 1e-9

    def test_yaw_only(self):
        """Test rotation around z-axis (yaw)."""
        yaw = np.pi / 4  # 45 degrees
        x, y, z, w = euler_to_quaternion(0.0, 0.0, yaw)

        # For pure yaw: x=0, y=0, z=sin(yaw/2), w=cos(yaw/2)
        assert abs(x) < 1e-9
        assert abs(y) < 1e-9
        assert abs(z - np.sin(yaw / 2)) < 1e-9
        assert abs(w - np.cos(yaw / 2)) < 1e-9

    def test_pitch_only(self):
        """Test rotation around y-axis (pitch)."""
        pitch = np.pi / 6  # 30 degrees
        x, y, z, w = euler_to_quaternion(0.0, pitch, 0.0)

        assert abs(x) < 1e-9
        assert abs(y - np.sin(pitch / 2)) < 1e-9
        assert abs(z) < 1e-9
        assert abs(w - np.cos(pitch / 2)) < 1e-9

    def test_roll_only(self):
        """Test rotation around x-axis (roll)."""
        roll = np.pi / 3  # 60 degrees
        x, y, z, w = euler_to_quaternion(roll, 0.0, 0.0)

        assert abs(x - np.sin(roll / 2)) < 1e-9
        assert abs(y) < 1e-9
        assert abs(z) < 1e-9
        assert abs(w - np.cos(roll / 2)) < 1e-9

    def test_euler_to_quat_roundtrip(self):
        """Test that Euler→Quat→Euler preserves angles."""
        roll_orig = 0.1
        pitch_orig = 0.2
        yaw_orig = 0.3

        x, y, z, w = euler_to_quaternion(roll_orig, pitch_orig, yaw_orig)
        roll_back, pitch_back, yaw_back = quaternion_to_euler(x, y, z, w)

        assert abs(roll_back - roll_orig) < 1e-9
        assert abs(pitch_back - pitch_orig) < 1e-9
        assert abs(yaw_back - yaw_orig) < 1e-9

    def test_quat_to_euler_roundtrip(self):
        """Test that Quat→Euler→Quat preserves quaternion."""
        # Create a properly normalized quaternion
        x_orig, y_orig, z_orig, w_orig = 0.1, 0.2, 0.3, 0.928
        mag = np.sqrt(x_orig**2 + y_orig**2 + z_orig**2 + w_orig**2)
        x_orig, y_orig, z_orig, w_orig = x_orig/mag, y_orig/mag, z_orig/mag, w_orig/mag

        roll, pitch, yaw = quaternion_to_euler(x_orig, y_orig, z_orig, w_orig)
        x_back, y_back, z_back, w_back = euler_to_quaternion(roll, pitch, yaw)

        assert abs(x_back - x_orig) < 1e-6
        assert abs(y_back - y_orig) < 1e-6
        assert abs(z_back - z_orig) < 1e-6
        assert abs(w_back - w_orig) < 1e-6

    def test_quaternion_normalization(self):
        """Test that generated quaternions are normalized."""
        for _ in range(10):
            roll = np.random.uniform(-np.pi, np.pi)
            pitch = np.random.uniform(-np.pi/2, np.pi/2)
            yaw = np.random.uniform(-np.pi, np.pi)

            x, y, z, w = euler_to_quaternion(roll, pitch, yaw)

            magnitude = np.sqrt(x*x + y*y + z*z + w*w)
            assert abs(magnitude - 1.0) < 1e-9


@pytest.mark.unit
class TestAngleNormalization:
    """Test angle normalization."""

    def test_zero_angle(self):
        """Test that zero angle remains zero."""
        assert normalize_angle(0.0) == 0.0

    def test_small_positive_angle(self):
        """Test that small positive angles are unchanged."""
        angle = 1.0
        assert abs(normalize_angle(angle) - angle) < 1e-9

    def test_small_negative_angle(self):
        """Test that small negative angles are unchanged."""
        angle = -1.0
        assert abs(normalize_angle(angle) - angle) < 1e-9

    def test_angle_greater_than_pi(self):
        """Test that angles > π are wrapped to negative."""
        angle = np.pi + 0.5
        normalized = normalize_angle(angle)

        assert normalized < 0
        assert abs(normalized - (-np.pi + 0.5)) < 1e-9

    def test_angle_less_than_minus_pi(self):
        """Test that angles < -π are wrapped to positive."""
        angle = -np.pi - 0.5
        normalized = normalize_angle(angle)

        assert normalized > 0
        assert abs(normalized - (np.pi - 0.5)) < 1e-9

    def test_two_pi_wraps_to_zero(self):
        """Test that 2π wraps to 0."""
        assert abs(normalize_angle(2 * np.pi)) < 1e-9

    def test_minus_two_pi_wraps_to_zero(self):
        """Test that -2π wraps to 0."""
        assert abs(normalize_angle(-2 * np.pi)) < 1e-9

    def test_multiple_rotations(self):
        """Test that multiple full rotations are handled."""
        angle = 10 * np.pi + 0.3  # 5 full rotations + 0.3
        normalized = normalize_angle(angle)

        assert -np.pi <= normalized <= np.pi
        assert abs(normalized - 0.3) < 1e-9
