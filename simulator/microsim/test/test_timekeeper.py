"""Unit tests for Timekeeper module."""

import pytest
from microsim.timekeeper import Timekeeper


@pytest.mark.unit
class TestTimekeeper:
    """Test fixed-step time management."""

    def test_initialization(self):
        """Test timekeeper initializes with correct defaults."""
        tk = Timekeeper()
        assert tk.dt == 1.0 / 60.0
        assert tk.target_rtf == 1.0
        assert tk.sim_time == 0.0
        assert tk.step_count == 0

    def test_custom_timestep(self):
        """Test timekeeper with custom timestep."""
        dt = 0.01
        tk = Timekeeper(dt=dt)
        assert tk.dt == dt

    def test_step_increments_time(self):
        """Test that step() advances simulation time."""
        tk = Timekeeper(dt=0.1)

        time1 = tk.step()
        assert time1 == 0.1
        assert tk.sim_time == 0.1
        assert tk.step_count == 1

        time2 = tk.step()
        assert time2 == 0.2
        assert tk.step_count == 2

    def test_step_precision(self):
        """Test that multiple steps maintain precision."""
        tk = Timekeeper(dt=1.0/60.0)

        # Run 600 steps (10 seconds)
        for _ in range(600):
            tk.step()

        # Should be exactly 10 seconds (within floating point tolerance)
        assert abs(tk.sim_time - 10.0) < 1e-10
        assert tk.step_count == 600

    def test_reset(self):
        """Test that reset() returns timekeeper to initial state."""
        tk = Timekeeper()

        # Advance time
        for _ in range(10):
            tk.step()

        assert tk.sim_time > 0
        assert tk.step_count > 0

        # Reset
        tk.reset()

        assert tk.sim_time == 0.0
        assert tk.step_count == 0

    def test_get_time(self):
        """Test get_time() returns current simulation time."""
        tk = Timekeeper(dt=0.1)
        assert tk.get_time() == 0.0

        tk.step()
        assert tk.get_time() == 0.1

    def test_get_step_count(self):
        """Test get_step_count() returns correct count."""
        tk = Timekeeper()
        assert tk.get_step_count() == 0

        tk.step()
        tk.step()
        assert tk.get_step_count() == 2

    def test_deterministic_sequence(self):
        """Test that two timekeepers produce identical sequences."""
        tk1 = Timekeeper(dt=1.0/60.0)
        tk2 = Timekeeper(dt=1.0/60.0)

        for _ in range(100):
            t1 = tk1.step()
            t2 = tk2.step()
            assert t1 == t2
