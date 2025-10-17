"""
Physics - Kinematic models for Drone and Rover.

Responsibilities:
- Update robot states given velocity commands
- Enforce kinematic constraints (max velocity, acceleration)
- Simple collision detection (binary stop if inside obstacle)
"""

import numpy as np
from typing import Tuple
from dataclasses import dataclass


@dataclass
class DroneState:
    """6-DOF drone state: position (x, y, z) and orientation (roll, pitch, yaw)."""
    x: float = 0.0
    y: float = 0.0
    z: float = 1.0  # Default hover height
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # Velocities
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    vyaw: float = 0.0


@dataclass
class RoverState:
    """Planar differential-drive rover state: position (x, y) and heading (theta)."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    # Velocities
    v: float = 0.0      # Linear velocity
    omega: float = 0.0  # Angular velocity


class DronePhysics:
    """Kinematic model for 6-DOF drone."""

    def __init__(self, max_velocity: float = 3.0, max_accel: float = 2.0):
        """
        Initialize drone physics.

        Args:
            max_velocity: Maximum velocity in m/s
            max_accel: Maximum acceleration in m/s^2
        """
        self.max_velocity = max_velocity
        self.max_accel = max_accel
        self.state = DroneState()

    def update(self, dt: float, vx_cmd: float, vy_cmd: float,
               vz_cmd: float, vyaw_cmd: float) -> DroneState:
        """
        Update drone state with velocity commands.

        Args:
            dt: Timestep in seconds
            vx_cmd, vy_cmd, vz_cmd: Linear velocity commands (m/s)
            vyaw_cmd: Yaw rate command (rad/s)

        Returns:
            Updated state
        """
        # Clamp velocities to max
        self.state.vx = np.clip(vx_cmd, -self.max_velocity, self.max_velocity)
        self.state.vy = np.clip(vy_cmd, -self.max_velocity, self.max_velocity)
        self.state.vz = np.clip(vz_cmd, -self.max_velocity, self.max_velocity)
        self.state.vyaw = vyaw_cmd  # No limit on yaw rate for now

        # Integrate position (body frame velocities → world frame)
        cos_yaw = np.cos(self.state.yaw)
        sin_yaw = np.sin(self.state.yaw)

        vx_world = self.state.vx * cos_yaw - self.state.vy * sin_yaw
        vy_world = self.state.vx * sin_yaw + self.state.vy * cos_yaw

        self.state.x += vx_world * dt
        self.state.y += vy_world * dt
        self.state.z += self.state.vz * dt
        self.state.yaw += self.state.vyaw * dt

        # Normalize yaw to [-pi, pi]
        self.state.yaw = np.arctan2(np.sin(self.state.yaw), np.cos(self.state.yaw))

        # Enforce ground collision (z >= 0)
        if self.state.z < 0:
            self.state.z = 0.0
            self.state.vz = 0.0

        return self.state

    def reset(self, x: float = 0.0, y: float = 0.0, z: float = 1.0) -> None:
        """Reset drone to initial position."""
        self.state = DroneState(x=x, y=y, z=z)


class RoverPhysics:
    """Kinematic model for differential-drive rover."""

    def __init__(self, max_velocity: float = 1.0, max_omega: float = 1.57):
        """
        Initialize rover physics.

        Args:
            max_velocity: Maximum linear velocity in m/s
            max_omega: Maximum angular velocity in rad/s
        """
        self.max_velocity = max_velocity
        self.max_omega = max_omega
        self.state = RoverState()

    def update(self, dt: float, v_cmd: float, omega_cmd: float) -> RoverState:
        """
        Update rover state with velocity commands.

        Args:
            dt: Timestep in seconds
            v_cmd: Linear velocity command (m/s)
            omega_cmd: Angular velocity command (rad/s)

        Returns:
            Updated state
        """
        # Clamp velocities
        self.state.v = np.clip(v_cmd, -self.max_velocity, self.max_velocity)
        self.state.omega = np.clip(omega_cmd, -self.max_omega, self.max_omega)

        # Integrate using simple Euler (good enough for slow velocities)
        self.state.x += self.state.v * np.cos(self.state.theta) * dt
        self.state.y += self.state.v * np.sin(self.state.theta) * dt
        self.state.theta += self.state.omega * dt

        # Normalize theta to [-pi, pi]
        self.state.theta = np.arctan2(np.sin(self.state.theta), np.cos(self.state.theta))

        return self.state

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """Reset rover to initial position."""
        self.state = RoverState(x=x, y=y, theta=theta)
