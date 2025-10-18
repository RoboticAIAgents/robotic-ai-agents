"""
TF Broadcaster - Publishes transform tree for robots.

Responsibilities:
- Publish transforms for world → drone, world → rover
- Publish camera frame transforms
- Maintain proper TF tree structure
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np


class TFBroadcaster:
    """Broadcasts TF transforms for robot frames."""

    def __init__(self, node: Node):
        """
        Initialize TF broadcaster.

        Args:
            node: ROS 2 node instance
        """
        self.node = node
        self.broadcaster = tf2_ros.TransformBroadcaster(node)

    def publish_drone_tf(self, timestamp, x: float, y: float, z: float,
                        roll: float, pitch: float, yaw: float) -> None:
        """
        Publish drone base_link transform.

        Args:
            timestamp: ROS time
            x, y, z: Position in world frame
            roll, pitch, yaw: Orientation in radians
        """
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'drone/base_link'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        # Convert RPY to quaternion
        quat = self._euler_to_quaternion(roll, pitch, yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(t)

    def publish_rover_tf(self, timestamp, x: float, y: float, theta: float) -> None:
        """
        Publish rover base_link transform.

        Args:
            timestamp: ROS time
            x, y: Position in world frame
            theta: Heading in radians
        """
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'rover/base_link'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # Quaternion for 2D rotation (yaw only)
        quat = self._euler_to_quaternion(0.0, 0.0, theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(t)

    def publish_camera_tf(self, timestamp) -> None:
        """
        Publish static camera frame (relative to drone base_link).

        Args:
            timestamp: ROS time
        """
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'drone/base_link'
        t.child_frame_id = 'drone/camera_link'

        # Camera mounted forward and above the body (clear of propellers)
        t.transform.translation.x = 0.15  # 15cm forward
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.05  # 5cm above body (not below)

        # Camera pitched down 15 degrees
        quat = self._euler_to_quaternion(0.0, np.radians(15.0), 0.0)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(t)

    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> tuple:
        """Convert Euler angles to quaternion (x, y, z, w)."""
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
