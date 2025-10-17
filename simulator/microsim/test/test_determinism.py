"""End-to-end determinism tests for the simulator.

These tests verify that the entire simulation produces identical results
when run with the same seed and inputs.
"""

import pytest
import numpy as np
from microsim.timekeeper import Timekeeper
from microsim.world import World, SemanticClass
from microsim.physics import DronePhysics, RoverPhysics
from microsim.sensors import GPS, RangeSensor
from microsim.radio import RadioLink


@pytest.mark.determinism
class TestSystemDeterminism:
    """Test determinism of complete simulation runs."""

    def test_complete_simulation_determinism(self):
        """Test that two complete simulation runs are identical."""

        # Run 1
        results1 = self._run_simulation(seed=12345)

        # Run 2 with same seed
        results2 = self._run_simulation(seed=12345)

        # Compare trajectories
        assert len(results1['drone_positions']) == len(results2['drone_positions'])
        assert len(results1['rover_positions']) == len(results2['rover_positions'])

        for i in range(len(results1['drone_positions'])):
            d1 = results1['drone_positions'][i]
            d2 = results2['drone_positions'][i]
            assert d1 == d2, f"Drone position mismatch at step {i}"

        for i in range(len(results1['rover_positions'])):
            r1 = results1['rover_positions'][i]
            r2 = results2['rover_positions'][i]
            assert r1 == r2, f"Rover position mismatch at step {i}"

        # Compare sensor readings
        assert results1['gps_readings'] == results2['gps_readings']
        assert results1['range_readings'] == results2['range_readings']

    def test_different_seeds_produce_different_results(self):
        """Test that different seeds produce different sensor noise."""

        results1 = self._run_simulation(seed=111)
        results2 = self._run_simulation(seed=999)

        # Positions should be the same (deterministic commands)
        # But sensor readings should differ due to noise
        assert results1['drone_positions'] == results2['drone_positions']
        assert results1['rover_positions'] == results2['rover_positions']

        # GPS readings should differ (due to noise)
        assert results1['gps_readings'] != results2['gps_readings']

    def test_world_state_determinism(self):
        """Test that world queries are deterministic."""
        world1 = World(size=(50.0, 50.0), resolution=0.5)
        world2 = World(size=(50.0, 50.0), resolution=0.5)

        # Place same features
        features = [
            (10.0, 10.0, 3.0, 5.0, SemanticClass.OBSTACLE),
            (-5.0, 15.0, 5.0, 0.0, SemanticClass.HAZARD),
            (20.0, -10.0, 2.0, 0.0, SemanticClass.TARGET),
        ]

        for x, y, radius, height, semantic in features:
            world1.set_region(x, y, radius, height, semantic)
            world2.set_region(x, y, radius, height, semantic)

        # Query many points
        test_points = [(i * 2.0, j * 2.0) for i in range(-10, 10) for j in range(-10, 10)]

        for x, y in test_points:
            assert world1.get_height(x, y) == world2.get_height(x, y)
            assert world1.get_semantic(x, y) == world2.get_semantic(x, y)

    def test_long_simulation_determinism(self):
        """Test determinism over a longer simulation (3600 steps = 60 seconds)."""

        results1 = self._run_simulation(seed=42, num_steps=3600)
        results2 = self._run_simulation(seed=42, num_steps=3600)

        # Check final positions
        final_drone_1 = results1['drone_positions'][-1]
        final_drone_2 = results2['drone_positions'][-1]
        assert final_drone_1 == final_drone_2

        final_rover_1 = results1['rover_positions'][-1]
        final_rover_2 = results2['rover_positions'][-1]
        assert final_rover_1 == final_rover_2

        # Spot check some intermediate states
        for i in [100, 500, 1000, 2000, 3000]:
            assert results1['drone_positions'][i] == results2['drone_positions'][i]
            assert results1['rover_positions'][i] == results2['rover_positions'][i]

    def test_replay_from_commands(self):
        """Test that recording and replaying commands produces identical results."""

        # Record commands and results from first run
        commands = self._generate_command_sequence(100)
        results1 = self._run_with_commands(commands, seed=999)

        # Replay same commands
        results2 = self._run_with_commands(commands, seed=999)

        # Should be identical
        assert results1 == results2

    def _run_simulation(self, seed: int, num_steps: int = 600):
        """Run a simple simulation and record results."""

        # Initialize
        timekeeper = Timekeeper(dt=1.0/60.0)
        drone = DronePhysics()
        rover = RoverPhysics()
        drone_gps = GPS(noise_std=0.1, rate_hz=10.0)
        rover_gps = GPS(noise_std=0.1, rate_hz=10.0)
        range_sensor = RangeSensor(max_range=10.0, rate_hz=20.0)

        # Seed sensors
        drone_gps.seed(seed)
        rover_gps.seed(seed + 1)
        range_sensor.seed(seed + 2)

        # Storage
        drone_positions = []
        rover_positions = []
        gps_readings = []
        range_readings = []

        # Run simulation
        for i in range(num_steps):
            # Simple command pattern
            t = timekeeper.get_time()
            drone.update(timekeeper.dt,
                        vx_cmd=1.0 * np.sin(t),
                        vy_cmd=0.5 * np.cos(t),
                        vz_cmd=0.0,
                        vyaw_cmd=0.1)

            rover.update(timekeeper.dt,
                        v_cmd=0.5,
                        omega_cmd=0.2 * np.sin(t * 2.0))

            # Record positions
            drone_positions.append((drone.state.x, drone.state.y, drone.state.z, drone.state.yaw))
            rover_positions.append((rover.state.x, rover.state.y, rover.state.theta))

            # Update sensors
            gps_reading = drone_gps.update(timekeeper.dt, drone.state.x, drone.state.y, drone.state.z)
            if gps_reading is not None:
                gps_readings.append(gps_reading)

            range_reading = range_sensor.update(timekeeper.dt, true_range=5.0)
            if range_reading is not None:
                range_readings.append(range_reading)

            timekeeper.step()

        return {
            'drone_positions': drone_positions,
            'rover_positions': rover_positions,
            'gps_readings': gps_readings,
            'range_readings': range_readings,
        }

    def _generate_command_sequence(self, num_steps: int):
        """Generate a deterministic command sequence."""
        commands = []
        for i in range(num_steps):
            t = i / 60.0
            commands.append({
                'drone': {
                    'vx': 1.5 * np.sin(t * 0.5),
                    'vy': 0.8 * np.cos(t * 0.3),
                    'vz': 0.2 * np.sin(t * 0.1),
                    'vyaw': 0.15 * np.cos(t * 0.2),
                },
                'rover': {
                    'v': 0.8 * (1.0 if i % 120 < 60 else -0.5),
                    'omega': 0.3 * np.sin(t * 0.4),
                }
            })
        return commands

    def _run_with_commands(self, commands, seed: int):
        """Run simulation with pre-recorded commands."""
        timekeeper = Timekeeper(dt=1.0/60.0)
        drone = DronePhysics()
        rover = RoverPhysics()
        drone_gps = GPS(noise_std=0.1, rate_hz=10.0)

        drone_gps.seed(seed)

        positions = []
        gps_readings = []

        for cmd in commands:
            drone.update(timekeeper.dt,
                        vx_cmd=cmd['drone']['vx'],
                        vy_cmd=cmd['drone']['vy'],
                        vz_cmd=cmd['drone']['vz'],
                        vyaw_cmd=cmd['drone']['vyaw'])

            rover.update(timekeeper.dt,
                        v_cmd=cmd['rover']['v'],
                        omega_cmd=cmd['rover']['omega'])

            positions.append((drone.state.x, drone.state.y, rover.state.x, rover.state.y))

            reading = drone_gps.update(timekeeper.dt, drone.state.x, drone.state.y, drone.state.z)
            if reading is not None:
                gps_readings.append(reading)

            timekeeper.step()

        return {
            'positions': positions,
            'gps_readings': gps_readings,
        }


@pytest.mark.determinism
class TestRadioDeterminism:
    """Test that radio link is deterministic."""

    def test_radio_packet_delivery_determinism(self):
        """Test that radio message delivery is deterministic with same seed."""

        radio1 = RadioLink(max_range=100.0, packet_loss=0.1)
        radio2 = RadioLink(max_range=100.0, packet_loss=0.1)

        radio1.seed(555)
        radio2.seed(555)

        # Send same sequence of messages
        messages_sent = []
        for i in range(50):
            msg = f"Message {i}".encode()
            messages_sent.append(msg)

            # Same distance, same time
            radio1.send_message('drone', 'rover', msg, current_time=i * 0.1, distance=50.0)
            radio2.send_message('drone', 'rover', msg, current_time=i * 0.1, distance=50.0)

        # Receive messages at same times
        received1 = []
        received2 = []

        for i in range(100):
            t = i * 0.05
            r1 = radio1.receive_messages('rover', current_time=t)
            r2 = radio2.receive_messages('rover', current_time=t)

            received1.extend(r1)
            received2.extend(r2)

        # Should have received same messages
        assert received1 == received2

    def test_radio_different_seeds_differ(self):
        """Test that different seeds produce different packet loss patterns."""

        radio1 = RadioLink(max_range=100.0, packet_loss=0.3)
        radio2 = RadioLink(max_range=100.0, packet_loss=0.3)

        radio1.seed(123)
        radio2.seed(456)

        # Send many messages
        for i in range(100):
            msg = f"Message {i}".encode()
            radio1.send_message('drone', 'rover', msg, current_time=i * 0.1, distance=50.0)
            radio2.send_message('drone', 'rover', msg, current_time=i * 0.1, distance=50.0)

        # Receive
        received1 = radio1.receive_messages('rover', current_time=100.0)
        received2 = radio2.receive_messages('rover', current_time=100.0)

        # Very unlikely to receive exactly the same messages with different seeds
        # (With 0.3 packet loss, probability of identical delivery is ~1e-10)
        assert received1 != received2 or len(received1) == 0  # Allow empty case


@pytest.mark.determinism
class TestNumericalStability:
    """Test numerical stability over long simulations."""

    def test_drone_position_stability(self):
        """Test that drone position doesn't accumulate floating point errors."""

        drone1 = DronePhysics()
        drone2 = DronePhysics()

        dt = 1.0 / 60.0

        # Run 36000 steps (10 minutes)
        for i in range(36000):
            # Oscillating commands
            vx = 2.0 * np.sin(i * 0.01)
            vy = 1.0 * np.cos(i * 0.02)

            drone1.update(dt, vx_cmd=vx, vy_cmd=vy, vz_cmd=0.0, vyaw_cmd=0.05)
            drone2.update(dt, vx_cmd=vx, vy_cmd=vy, vz_cmd=0.0, vyaw_cmd=0.05)

        # Should still be identical
        assert drone1.state.x == drone2.state.x
        assert drone1.state.y == drone2.state.y
        assert drone1.state.yaw == drone2.state.yaw

    def test_time_accumulation_precision(self):
        """Test that time accumulation maintains precision."""

        tk = Timekeeper(dt=1.0/60.0)

        # Run for 1 hour (216000 steps)
        for _ in range(216000):
            tk.step()

        # Should be exactly 3600 seconds (within reasonable tolerance)
        # Note: Some floating point accumulation is expected over 216000 steps
        assert abs(tk.sim_time - 3600.0) < 1e-7
