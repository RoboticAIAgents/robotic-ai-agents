"""Unit tests for Sensors module."""

import pytest
import numpy as np
from microsim.sensors import GPS, RangeSensor


@pytest.mark.unit
class TestGPS:
    """Test GPS sensor."""

    def test_initialization(self):
        """Test GPS initializes with correct parameters."""
        gps = GPS(noise_std=0.1, rate_hz=10.0)
        assert gps.noise_std == 0.1
        assert gps.rate_hz == 10.0
        assert gps.dt == 0.1
        assert gps.time_since_update == 0.0

    def test_update_timing(self):
        """Test GPS only publishes at specified rate."""
        gps = GPS(rate_hz=10.0)  # 10 Hz = 0.1s period

        # First update should return None (not ready)
        reading = gps.update(dt=0.05, x=1.0, y=2.0, z=3.0)
        assert reading is None

        # After another 0.05s (total 0.1s), should publish
        reading = gps.update(dt=0.05, x=1.0, y=2.0, z=3.0)
        assert reading is not None
        assert len(reading) == 3

    def test_adds_noise(self):
        """Test that GPS adds noise to measurements."""
        gps = GPS(noise_std=0.5, rate_hz=10.0)
        gps.seed(42)  # Seed for reproducibility

        # Update until we get a reading
        reading = gps.update(dt=0.1, x=10.0, y=20.0, z=5.0)

        # Should be noisy (not exact)
        x, y, z = reading
        assert x != 10.0 or y != 20.0 or z != 5.0

        # Should be close though (within a few sigma)
        assert abs(x - 10.0) < 2.0  # ~4 std deviations
        assert abs(y - 20.0) < 2.0
        assert abs(z - 5.0) < 2.0

    def test_noise_is_zero_mean(self):
        """Test that GPS noise averages to zero over many samples."""
        gps = GPS(noise_std=0.1, rate_hz=100.0)
        gps.seed(123)

        true_x, true_y, true_z = 5.0, 10.0, 2.0
        measurements_x = []
        measurements_y = []
        measurements_z = []

        # Collect 1000 samples
        for _ in range(1000):
            reading = gps.update(dt=0.01, x=true_x, y=true_y, z=true_z)
            if reading is not None:
                x, y, z = reading
                measurements_x.append(x - true_x)
                measurements_y.append(y - true_y)
                measurements_z.append(z - true_z)

        # Mean error should be close to zero
        assert abs(np.mean(measurements_x)) < 0.02
        assert abs(np.mean(measurements_y)) < 0.02
        assert abs(np.mean(measurements_z)) < 0.02

    def test_seeding(self):
        """Test that seeding produces deterministic noise."""
        gps1 = GPS(noise_std=0.1, rate_hz=10.0)
        gps2 = GPS(noise_std=0.1, rate_hz=10.0)

        gps1.seed(42)
        gps2.seed(42)

        # Both should produce identical readings
        for _ in range(10):
            r1 = gps1.update(dt=0.1, x=1.0, y=2.0, z=3.0)
            r2 = gps2.update(dt=0.1, x=1.0, y=2.0, z=3.0)

            if r1 is not None and r2 is not None:
                assert r1 == r2

    def test_different_seeds_produce_different_noise(self):
        """Test that different seeds produce different noise."""
        gps1 = GPS(noise_std=0.1, rate_hz=10.0)
        gps2 = GPS(noise_std=0.1, rate_hz=10.0)

        gps1.seed(42)
        gps2.seed(99)

        r1 = gps1.update(dt=0.1, x=1.0, y=2.0, z=3.0)
        r2 = gps2.update(dt=0.1, x=1.0, y=2.0, z=3.0)

        # Very unlikely to be identical
        assert r1 != r2

    def test_low_rate(self):
        """Test GPS with low update rate."""
        gps = GPS(rate_hz=1.0)  # 1 Hz = 1.0s period

        # Should NOT be ready before 1 second
        reading = gps.update(dt=0.9, x=1.0, y=2.0, z=3.0)
        assert reading is None

        # After 1 second total, should be ready
        reading = gps.update(dt=0.1, x=1.0, y=2.0, z=3.0)
        assert reading is not None


@pytest.mark.unit
class TestRangeSensor:
    """Test forward range sensor."""

    def test_initialization(self):
        """Test range sensor initializes correctly."""
        sensor = RangeSensor(max_range=10.0, noise_std=0.05, rate_hz=20.0)
        assert sensor.max_range == 10.0
        assert sensor.noise_std == 0.05
        assert sensor.rate_hz == 20.0
        assert sensor.dt == 0.05

    def test_update_timing(self):
        """Test range sensor publishes at correct rate."""
        sensor = RangeSensor(rate_hz=20.0)  # 20 Hz = 0.05s period

        # Not ready yet
        reading = sensor.update(dt=0.02, true_range=5.0)
        assert reading is None

        # Still not ready
        reading = sensor.update(dt=0.02, true_range=5.0)
        assert reading is None

        # Now ready (total 0.04s + 0.01s = 0.05s)
        reading = sensor.update(dt=0.01, true_range=5.0)
        assert reading is not None

    def test_adds_noise(self):
        """Test that range sensor adds noise."""
        sensor = RangeSensor(noise_std=0.1, rate_hz=20.0)
        sensor.seed(42)

        reading = sensor.update(dt=0.05, true_range=5.0)

        # Should be noisy
        assert reading != 5.0
        assert abs(reading - 5.0) < 0.5  # Within reasonable bounds

    def test_clamps_to_zero(self):
        """Test that negative ranges are clamped to zero."""
        sensor = RangeSensor(noise_std=10.0, rate_hz=20.0)  # High noise
        sensor.seed(42)

        # With high noise, negative reading is possible
        # Sensor should clamp to 0
        reading = sensor.update(dt=0.05, true_range=0.1)
        assert reading >= 0.0

    def test_clamps_to_max_range(self):
        """Test that readings are clamped to max_range."""
        sensor = RangeSensor(max_range=5.0, noise_std=0.01, rate_hz=20.0)
        sensor.seed(42)

        # True range beyond max
        reading = sensor.update(dt=0.05, true_range=100.0)

        assert reading <= 5.0

    def test_seeding(self):
        """Test that seeding produces deterministic results."""
        sensor1 = RangeSensor(noise_std=0.1, rate_hz=20.0)
        sensor2 = RangeSensor(noise_std=0.1, rate_hz=20.0)

        sensor1.seed(42)
        sensor2.seed(42)

        for _ in range(10):
            r1 = sensor1.update(dt=0.05, true_range=3.0)
            r2 = sensor2.update(dt=0.05, true_range=3.0)

            if r1 is not None and r2 is not None:
                assert r1 == r2

    def test_zero_noise(self):
        """Test range sensor with no noise returns exact values."""
        sensor = RangeSensor(noise_std=0.0, rate_hz=20.0)

        reading = sensor.update(dt=0.05, true_range=7.5)
        assert reading == 7.5

    def test_noise_statistics(self):
        """Test that noise has correct statistics."""
        sensor = RangeSensor(noise_std=0.1, rate_hz=100.0)
        sensor.seed(123)

        true_range = 5.0
        measurements = []

        # Collect many samples
        for _ in range(1000):
            reading = sensor.update(dt=0.01, true_range=true_range)
            if reading is not None:
                measurements.append(reading)

        # Mean should be close to true range
        assert abs(np.mean(measurements) - true_range) < 0.02

        # Standard deviation should be close to noise_std
        assert abs(np.std(measurements) - 0.1) < 0.02

    def test_max_range_edge_case(self):
        """Test behavior exactly at max range."""
        sensor = RangeSensor(max_range=10.0, noise_std=0.0, rate_hz=20.0)

        reading = sensor.update(dt=0.05, true_range=10.0)
        assert reading == 10.0


@pytest.mark.determinism
class TestSensorDeterminism:
    """Test that sensors produce deterministic outputs when seeded."""

    def test_gps_deterministic_sequence(self):
        """Test GPS produces identical sequence with same seed."""
        gps1 = GPS(noise_std=0.2, rate_hz=10.0)
        gps2 = GPS(noise_std=0.2, rate_hz=10.0)

        gps1.seed(777)
        gps2.seed(777)

        # Simulate 50 steps
        for i in range(50):
            x = i * 0.1
            y = i * 0.2
            z = 5.0 + i * 0.05

            r1 = gps1.update(dt=0.01, x=x, y=y, z=z)
            r2 = gps2.update(dt=0.01, x=x, y=y, z=z)

            if r1 is not None and r2 is not None:
                assert r1 == r2

    def test_range_deterministic_sequence(self):
        """Test range sensor produces identical sequence with same seed."""
        sensor1 = RangeSensor(noise_std=0.1, rate_hz=20.0)
        sensor2 = RangeSensor(noise_std=0.1, rate_hz=20.0)

        sensor1.seed(888)
        sensor2.seed(888)

        for i in range(100):
            true_range = 5.0 + np.sin(i * 0.1)

            r1 = sensor1.update(dt=0.01, true_range=true_range)
            r2 = sensor2.update(dt=0.01, true_range=true_range)

            if r1 is not None and r2 is not None:
                assert r1 == r2
