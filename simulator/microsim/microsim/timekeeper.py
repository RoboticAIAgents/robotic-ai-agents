"""
Timekeeper - Fixed-step simulation time management.

Responsibilities:
- Maintain simulation time with fixed dt (1/60 s)
- Enforce real-time factor constraints
- Provide step() interface for deterministic execution
"""

from typing import Optional


class Timekeeper:
    """Manages simulation time with fixed timestep and real-time constraints."""

    def __init__(self, dt: float = 1.0/60.0, target_rtf: float = 1.0):
        """
        Initialize timekeeper.

        Args:
            dt: Fixed timestep in seconds (default: 1/60)
            target_rtf: Target real-time factor (default: 1.0)
        """
        self.dt = dt
        self.target_rtf = target_rtf
        self.sim_time = 0.0
        self.step_count = 0

    def step(self) -> float:
        """
        Advance simulation by one timestep.

        Returns:
            Current simulation time after step
        """
        self.step_count += 1
        self.sim_time += self.dt
        return self.sim_time

    def reset(self) -> None:
        """Reset timekeeper to initial state."""
        self.sim_time = 0.0
        self.step_count = 0

    def get_time(self) -> float:
        """Get current simulation time."""
        return self.sim_time

    def get_step_count(self) -> int:
        """Get current step count."""
        return self.step_count
