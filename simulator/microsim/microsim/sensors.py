"""
Sensors - GPS and Range sensor implementations.

Responsibilities:
- GPS: Provide noisy position measurements
- Range: Forward-facing distance to nearest obstacle
"""

import numpy as np
from typing import Optional


class GPS:
    """GPS sensor providing noisy position measurements."""

    def __init__(self, noise_std: float = 0.1, rate_hz: float = 10.0):
        """
        Initialize GPS sensor.

        Args:
            noise_std: Position noise standard deviation in meters
            rate_hz: Sensor update rate in Hz
        """
        self.noise_std = noise_std
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.time_since_update = 0.0
        self.rng = np.random.default_rng(0)  # Will be seeded properly

    def update(self, dt: float, x: float, y: float, z: float) -> Optional[tuple]:
        """
        Update GPS sensor.

        Args:
            dt: Timestep in seconds
            x, y, z: True position

        Returns:
            (x_noisy, y_noisy, z_noisy) if ready, None otherwise
        """
        self.time_since_update += dt

        if self.time_since_update >= self.dt:
            self.time_since_update = 0.0

            # Add Gaussian noise
            noise = self.rng.normal(0.0, self.noise_std, size=3)
            return (x + noise[0], y + noise[1], z + noise[2])

        return None

    def seed(self, seed: int) -> None:
        """Seed random number generator."""
        self.rng = np.random.default_rng(seed)


class RangeSensor:
    """Forward-facing range sensor."""

    def __init__(self, max_range: float = 10.0, noise_std: float = 0.05,
                 rate_hz: float = 20.0):
        """
        Initialize range sensor.

        Args:
            max_range: Maximum detection range in meters
            noise_std: Range noise standard deviation in meters
            rate_hz: Sensor update rate in Hz
        """
        self.max_range = max_range
        self.noise_std = noise_std
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.time_since_update = 0.0
        self.rng = np.random.default_rng(0)

    def update(self, dt: float, true_range: float) -> Optional[float]:
        """
        Update range sensor.

        Args:
            dt: Timestep in seconds
            true_range: True range to obstacle in meters

        Returns:
            Noisy range measurement if ready, None otherwise
        """
        self.time_since_update += dt

        if self.time_since_update >= self.dt:
            self.time_since_update = 0.0

            # Add Gaussian noise
            noise = self.rng.normal(0.0, self.noise_std)
            measured = true_range + noise

            # Clamp to [0, max_range]
            return float(np.clip(measured, 0.0, self.max_range))

        return None

    def seed(self, seed: int) -> None:
        """Seed random number generator."""
        self.rng = np.random.default_rng(seed)
