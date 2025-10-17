"""Unit tests for Physics module."""

import pytest
import numpy as np
from microsim.physics import DronePhysics, RoverPhysics, DroneState, RoverState


@pytest.mark.unit
class TestDronePhysics:
    """Test 6-DOF drone kinematics."""

    def test_initialization(self):
        """Test drone initializes at origin with default params."""
        drone = DronePhysics()
        assert drone.state.x == 0.0
        assert drone.state.y == 0.0
        assert drone.state.z == 1.0  # Default hover height
        assert drone.state.yaw == 0.0
        assert drone.max_velocity == 3.0

    def test_forward_motion(self):
        """Test drone moves forward with positive vx."""
        drone = DronePhysics()
        dt = 0.1

        # Command 1 m/s forward
        drone.update(dt, vx_cmd=1.0, vy_cmd=0.0, vz_cmd=0.0, vyaw_cmd=0.0)

        # Should move 0.1m forward in x direction
        assert abs(drone.state.x - 0.1) < 1e-6
        assert abs(drone.state.y) < 1e-6
        assert drone.state.z == 1.0

    def test_lateral_motion(self):
        """Test drone moves laterally with vy."""
        drone = DronePhysics()
        dt = 0.1

        # Command 1 m/s to the left
        drone.update(dt, vx_cmd=0.0, vy_cmd=1.0, vz_cmd=0.0, vyaw_cmd=0.0)

        assert abs(drone.state.x) < 1e-6
        assert abs(drone.state.y - 0.1) < 1e-6

    def test_vertical_motion(self):
        """Test drone climbs/descends with vz."""
        drone = DronePhysics()
        dt = 0.1

        # Command 1 m/s upward
        drone.update(dt, vx_cmd=0.0, vy_cmd=0.0, vz_cmd=1.0, vyaw_cmd=0.0)

        assert abs(drone.state.z - 1.1) < 1e-6

    def test_yaw_rotation(self):
        """Test drone rotates with yaw command."""
        drone = DronePhysics()
        dt = 0.1

        # Command 1 rad/s yaw rate
        drone.update(dt, vx_cmd=0.0, vy_cmd=0.0, vz_cmd=0.0, vyaw_cmd=1.0)

        assert abs(drone.state.yaw - 0.1) < 1e-6

    def test_body_frame_velocity(self):
        """Test that velocity commands are in body frame."""
        drone = DronePhysics()
        dt = 0.1

        # Rotate 90 degrees
        drone.state.yaw = np.pi / 2

        # Command forward in body frame
        drone.update(dt, vx_cmd=1.0, vy_cmd=0.0, vz_cmd=0.0, vyaw_cmd=0.0)

        # Should move in -y world direction (since yaw=90deg)
        assert abs(drone.state.x) < 1e-6
        assert abs(drone.state.y - 0.1) < 1e-6

    def test_velocity_clamping(self):
        """Test that velocities are clamped to max."""
        drone = DronePhysics(max_velocity=2.0)
        dt = 0.1

        # Command higher than max
        drone.update(dt, vx_cmd=5.0, vy_cmd=0.0, vz_cmd=0.0, vyaw_cmd=0.0)

        assert drone.state.vx == 2.0  # Clamped to max
        assert abs(drone.state.x - 0.2) < 1e-6  # Moved at max velocity

    def test_ground_collision(self):
        """Test that drone can't go below z=0."""
        drone = DronePhysics()
        drone.state.z = 0.1
        dt = 0.1

        # Command downward faster than current height
        drone.update(dt, vx_cmd=0.0, vy_cmd=0.0, vz_cmd=-2.0, vyaw_cmd=0.0)

        # Should stop at z=0
        assert drone.state.z == 0.0
        assert drone.state.vz == 0.0

    def test_yaw_normalization(self):
        """Test that yaw is normalized to [-pi, pi]."""
        drone = DronePhysics()
        dt = 1.0

        # Spin more than 2*pi
        drone.update(dt, vx_cmd=0.0, vy_cmd=0.0, vz_cmd=0.0, vyaw_cmd=7.0)

        # Should be normalized
        assert -np.pi <= drone.state.yaw <= np.pi

    def test_reset(self):
        """Test reset returns drone to specified position."""
        drone = DronePhysics()

        # Move drone
        drone.update(0.1, vx_cmd=1.0, vy_cmd=1.0, vz_cmd=1.0, vyaw_cmd=1.0)
        assert drone.state.x != 0.0

        # Reset
        drone.reset(x=5.0, y=10.0, z=2.0)

        assert drone.state.x == 5.0
        assert drone.state.y == 10.0
        assert drone.state.z == 2.0
        assert drone.state.yaw == 0.0


@pytest.mark.unit
class TestRoverPhysics:
    """Test differential-drive rover kinematics."""

    def test_initialization(self):
        """Test rover initializes at origin."""
        rover = RoverPhysics()
        assert rover.state.x == 0.0
        assert rover.state.y == 0.0
        assert rover.state.theta == 0.0
        assert rover.max_velocity == 1.0

    def test_forward_motion(self):
        """Test rover moves forward."""
        rover = RoverPhysics()
        dt = 0.1

        # Command 1 m/s forward
        rover.update(dt, v_cmd=1.0, omega_cmd=0.0)

        # Should move 0.1m forward in x direction (theta=0)
        assert abs(rover.state.x - 0.1) < 1e-6
        assert abs(rover.state.y) < 1e-6

    def test_rotation(self):
        """Test rover rotates in place."""
        rover = RoverPhysics()
        dt = 0.1

        # Command 1 rad/s rotation, no translation
        rover.update(dt, v_cmd=0.0, omega_cmd=1.0)

        assert abs(rover.state.theta - 0.1) < 1e-6
        assert abs(rover.state.x) < 1e-6
        assert abs(rover.state.y) < 1e-6

    def test_curved_motion(self):
        """Test rover follows curved path."""
        rover = RoverPhysics()
        dt = 0.1

        # Move forward while turning
        rover.update(dt, v_cmd=1.0, omega_cmd=1.0)

        # Should have moved and rotated
        assert rover.state.x > 0
        assert abs(rover.state.theta - 0.1) < 1e-6

    def test_heading_affects_motion(self):
        """Test that rover motion follows heading."""
        rover = RoverPhysics()
        dt = 0.1

        # Face 90 degrees (north)
        rover.state.theta = np.pi / 2

        # Move forward
        rover.update(dt, v_cmd=1.0, omega_cmd=0.0)

        # Should move in y direction
        assert abs(rover.state.x) < 1e-6
        assert abs(rover.state.y - 0.1) < 1e-6

    def test_velocity_clamping(self):
        """Test that velocities are clamped."""
        rover = RoverPhysics(max_velocity=0.5, max_omega=1.0)
        dt = 0.1

        # Command higher than max
        rover.update(dt, v_cmd=2.0, omega_cmd=5.0)

        assert rover.state.v == 0.5
        assert rover.state.omega == 1.0

    def test_backward_motion(self):
        """Test rover can move backward."""
        rover = RoverPhysics()
        dt = 0.1

        # Command backward
        rover.update(dt, v_cmd=-1.0, omega_cmd=0.0)

        assert rover.state.x < 0

    def test_theta_normalization(self):
        """Test that theta is normalized to [-pi, pi]."""
        rover = RoverPhysics()
        dt = 1.0

        # Spin more than 2*pi
        rover.update(dt, v_cmd=0.0, omega_cmd=7.0)

        assert -np.pi <= rover.state.theta <= np.pi

    def test_reset(self):
        """Test reset returns rover to specified position."""
        rover = RoverPhysics()

        # Move rover
        rover.update(0.1, v_cmd=1.0, omega_cmd=1.0)

        # Reset
        rover.reset(x=10.0, y=5.0, theta=np.pi/4)

        assert rover.state.x == 10.0
        assert rover.state.y == 5.0
        assert rover.state.theta == np.pi/4
        assert rover.state.v == 0.0


@pytest.mark.unit
class TestDeterminism:
    """Test that physics updates are deterministic."""

    def test_drone_determinism(self):
        """Test two drones with same commands produce identical trajectories."""
        drone1 = DronePhysics()
        drone2 = DronePhysics()

        dt = 1.0 / 60.0

        for i in range(100):
            # Same commands
            vx = np.sin(i * 0.1)
            vy = np.cos(i * 0.1)

            drone1.update(dt, vx_cmd=vx, vy_cmd=vy, vz_cmd=0.0, vyaw_cmd=0.1)
            drone2.update(dt, vx_cmd=vx, vy_cmd=vy, vz_cmd=0.0, vyaw_cmd=0.1)

            assert drone1.state.x == drone2.state.x
            assert drone1.state.y == drone2.state.y
            assert drone1.state.yaw == drone2.state.yaw

    def test_rover_determinism(self):
        """Test two rovers with same commands produce identical trajectories."""
        rover1 = RoverPhysics()
        rover2 = RoverPhysics()

        dt = 1.0 / 60.0

        for i in range(100):
            v = 1.0 if i % 20 < 10 else -0.5
            omega = 0.5 * np.sin(i * 0.05)

            rover1.update(dt, v_cmd=v, omega_cmd=omega)
            rover2.update(dt, v_cmd=v, omega_cmd=omega)

            assert rover1.state.x == rover2.state.x
            assert rover1.state.y == rover2.state.y
            assert rover1.state.theta == rover2.state.theta
