"""
Radio - Simple radio link model for inter-robot communication.

Responsibilities:
- Model message transmission with latency, jitter, packet loss
- Distance-based signal degradation
- Maintain message queues for each robot
"""

import numpy as np
from typing import Optional, List
from dataclasses import dataclass
from collections import deque


@dataclass
class RadioMessage:
    """Message in flight through radio link."""
    payload: bytes
    delivery_time: float  # Sim time when message should arrive


class RadioLink:
    """Radio link model for inter-robot communication."""

    def __init__(self, max_range: float = 100.0, base_latency: float = 0.05,
                 jitter: float = 0.01, packet_loss: float = 0.01):
        """
        Initialize radio link.

        Args:
            max_range: Maximum communication range in meters
            base_latency: Base latency in seconds
            jitter: Latency jitter in seconds
            packet_loss: Packet loss probability [0, 1]
        """
        self.max_range = max_range
        self.base_latency = base_latency
        self.jitter = jitter
        self.packet_loss = packet_loss

        # Message queues (in-flight messages)
        self.drone_to_rover_queue: deque = deque()
        self.rover_to_drone_queue: deque = deque()

        self.rng = np.random.default_rng(0)

    def send_message(self, from_robot: str, to_robot: str, payload: bytes,
                    current_time: float, distance: float) -> bool:
        """
        Send message from one robot to another.

        Args:
            from_robot: Source robot ('drone' or 'rover')
            to_robot: Destination robot
            payload: Message payload
            current_time: Current simulation time
            distance: Distance between robots in meters

        Returns:
            True if message accepted, False if dropped
        """
        # Check range
        if distance > self.max_range:
            return False

        # Check packet loss
        if self.rng.random() < self.packet_loss:
            return False

        # Compute delivery time with jitter
        latency = self.base_latency + self.rng.uniform(-self.jitter, self.jitter)
        delivery_time = current_time + latency

        # Add to appropriate queue
        msg = RadioMessage(payload=payload, delivery_time=delivery_time)
        if from_robot == 'drone':
            self.drone_to_rover_queue.append(msg)
        else:
            self.rover_to_drone_queue.append(msg)

        return True

    def receive_messages(self, robot: str, current_time: float) -> List[bytes]:
        """
        Receive messages for a robot.

        Args:
            robot: Robot name ('drone' or 'rover')
            current_time: Current simulation time

        Returns:
            List of message payloads ready for delivery
        """
        queue = self.rover_to_drone_queue if robot == 'drone' else self.drone_to_rover_queue
        messages = []

        while queue and queue[0].delivery_time <= current_time:
            msg = queue.popleft()
            messages.append(msg.payload)

        return messages

    def seed(self, seed: int) -> None:
        """Seed random number generator."""
        self.rng = np.random.default_rng(seed)
