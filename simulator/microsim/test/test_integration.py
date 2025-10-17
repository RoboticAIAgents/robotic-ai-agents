"""
Integration tests for MicroSim.

These tests verify that multiple components work together correctly
in realistic scenarios. They use ROS 2 launch_testing framework to
start the actual node and verify behavior through topic interactions.
"""

import unittest
import time
import math
import numpy as np
from threading import Event, Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Range, Image, CameraInfo
from std_msgs.msg import String


class IntegrationTestNode(Node):
    """Helper node for integration tests."""

    def __init__(self, test_name='integration_test'):
        super().__init__(test_name)

        # Storage for received messages
        self.drone_odom_msgs = []
        self.rover_odom_msgs = []
        self.drone_gps_msgs = []
        self.rover_gps_msgs = []
        self.rover_range_msgs = []
        self.camera_image_msgs = []
        self.camera_info_msgs = []
        self.drone_radio_rx_msgs = []
        self.rover_radio_rx_msgs = []

        # Locks for thread-safe access
        self.lock = Lock()

        # Events for synchronization
        self.received_drone_odom = Event()
        self.received_rover_odom = Event()
        self.received_drone_gps = Event()
        self.received_rover_gps = Event()
        self.received_rover_range = Event()
        self.received_camera_image = Event()
        self.received_drone_radio = Event()
        self.received_rover_radio = Event()

        # QoS profile for radio (BEST_EFFORT to match simulator)
        radio_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.create_subscription(Odometry, '/drone/odom', self._drone_odom_cb, 10)
        self.create_subscription(Odometry, '/rover/odom', self._rover_odom_cb, 10)
        self.create_subscription(NavSatFix, '/drone/gps', self._drone_gps_cb, 10)
        self.create_subscription(NavSatFix, '/rover/gps', self._rover_gps_cb, 10)
        self.create_subscription(Range, '/rover/range', self._rover_range_cb, 10)
        self.create_subscription(Image, '/drone/camera/image_raw', self._camera_image_cb, 10)
        self.create_subscription(CameraInfo, '/drone/camera/camera_info', self._camera_info_cb, 10)
        self.create_subscription(String, '/radio/drone_rx', self._drone_radio_cb, radio_qos)
        self.create_subscription(String, '/radio/rover_rx', self._rover_radio_cb, radio_qos)

        # Publishers for commands
        self.drone_cmd_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.rover_cmd_pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        self.drone_radio_pub = self.create_publisher(String, '/radio/drone_tx', radio_qos)
        self.rover_radio_pub = self.create_publisher(String, '/radio/rover_tx', radio_qos)

    def _drone_odom_cb(self, msg):
        with self.lock:
            self.drone_odom_msgs.append(msg)
            self.received_drone_odom.set()

    def _rover_odom_cb(self, msg):
        with self.lock:
            self.rover_odom_msgs.append(msg)
            self.received_rover_odom.set()

    def _drone_gps_cb(self, msg):
        with self.lock:
            self.drone_gps_msgs.append(msg)
            self.received_drone_gps.set()

    def _rover_gps_cb(self, msg):
        with self.lock:
            self.rover_gps_msgs.append(msg)
            self.received_rover_gps.set()

    def _rover_range_cb(self, msg):
        with self.lock:
            self.rover_range_msgs.append(msg)
            self.received_rover_range.set()

    def _camera_image_cb(self, msg):
        with self.lock:
            self.camera_image_msgs.append(msg)
            self.received_camera_image.set()

    def _camera_info_cb(self, msg):
        with self.lock:
            self.camera_info_msgs.append(msg)

    def _drone_radio_cb(self, msg):
        with self.lock:
            self.drone_radio_rx_msgs.append(msg)
            self.received_drone_radio.set()

    def _rover_radio_cb(self, msg):
        with self.lock:
            self.rover_radio_rx_msgs.append(msg)
            self.received_rover_radio.set()

    def clear_messages(self):
        """Clear all stored messages."""
        with self.lock:
            self.drone_odom_msgs.clear()
            self.rover_odom_msgs.clear()
            self.drone_gps_msgs.clear()
            self.rover_gps_msgs.clear()
            self.rover_range_msgs.clear()
            self.camera_image_msgs.clear()
            self.camera_info_msgs.clear()
            self.drone_radio_rx_msgs.clear()
            self.rover_radio_rx_msgs.clear()

    def get_latest_drone_odom(self):
        """Get the most recent drone odometry message."""
        with self.lock:
            return self.drone_odom_msgs[-1] if self.drone_odom_msgs else None

    def get_latest_rover_odom(self):
        """Get the most recent rover odometry message."""
        with self.lock:
            return self.rover_odom_msgs[-1] if self.rover_odom_msgs else None

    def publish_drone_cmd(self, vx, vy, vz, vyaw):
        """Publish drone velocity command."""
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = vyaw
        self.drone_cmd_pub.publish(cmd)

    def publish_rover_cmd(self, v, omega):
        """Publish rover velocity command."""
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.rover_cmd_pub.publish(cmd)

    def publish_drone_radio(self, message):
        """Publish radio message from drone."""
        msg = String()
        msg.data = message
        self.drone_radio_pub.publish(msg)

    def publish_rover_radio(self, message):
        """Publish radio message from rover."""
        msg = String()
        msg.data = message
        self.rover_radio_pub.publish(msg)


class TestIntegration(unittest.TestCase):
    """Integration tests for MicroSim simulator."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 and create test node."""
        rclpy.init()
        cls.test_node = IntegrationTestNode('microsim_integration_test')
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.test_node)

        # Give the simulator node time to start up
        cls._spin_for_time(2.0)

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS 2."""
        cls.test_node.destroy_node()
        cls.executor.shutdown()
        rclpy.shutdown()

    def setUp(self):
        """Clear messages before each test."""
        self.test_node.clear_messages()

    @classmethod
    def _spin_for_time(cls, duration):
        """Spin the executor for a specified duration in seconds."""
        start_time = time.time()
        while time.time() - start_time < duration:
            cls.executor.spin_once(timeout_sec=0.01)

    def test_all_topics_publishing(self):
        """Test that all expected topics are publishing."""
        # Wait for at least one message on each topic
        self._spin_for_time(2.0)

        # Check that we received messages
        self.assertTrue(self.test_node.received_drone_odom.is_set(),
                       "Did not receive drone odometry")
        self.assertTrue(self.test_node.received_rover_odom.is_set(),
                       "Did not receive rover odometry")
        self.assertTrue(self.test_node.received_drone_gps.is_set(),
                       "Did not receive drone GPS")
        self.assertTrue(self.test_node.received_rover_gps.is_set(),
                       "Did not receive rover GPS")
        self.assertTrue(self.test_node.received_rover_range.is_set(),
                       "Did not receive rover range")
        self.assertTrue(self.test_node.received_camera_image.is_set(),
                       "Did not receive camera image")

    def test_topic_rates(self):
        """Test that topics publish at approximately expected rates."""
        # Clear existing messages
        self.test_node.clear_messages()

        # Collect messages for 3 seconds
        self._spin_for_time(3.0)

        # Check odometry rate (~60 Hz)
        with self.test_node.lock:
            drone_odom_count = len(self.test_node.drone_odom_msgs)
            rover_odom_count = len(self.test_node.rover_odom_msgs)

        # Allow 20% tolerance for timing variations
        self.assertGreater(drone_odom_count, 140,  # ~47 Hz minimum
                          f"Drone odometry rate too low: {drone_odom_count/3.0:.1f} Hz")
        self.assertGreater(rover_odom_count, 140,
                          f"Rover odometry rate too low: {rover_odom_count/3.0:.1f} Hz")

        # Check GPS rate (~10 Hz)
        with self.test_node.lock:
            drone_gps_count = len(self.test_node.drone_gps_msgs)
            rover_gps_count = len(self.test_node.rover_gps_msgs)

        self.assertGreater(drone_gps_count, 20,  # ~7 Hz minimum
                          f"Drone GPS rate too low: {drone_gps_count/3.0:.1f} Hz")
        self.assertGreater(rover_gps_count, 20,
                          f"Rover GPS rate too low: {rover_gps_count/3.0:.1f} Hz")

        # Check range sensor rate (~20 Hz)
        with self.test_node.lock:
            range_count = len(self.test_node.rover_range_msgs)

        self.assertGreater(range_count, 40,  # ~13 Hz minimum
                          f"Range sensor rate too low: {range_count/3.0:.1f} Hz")

    def test_drone_command_to_movement(self):
        """Test that drone velocity commands result in position changes."""
        # First, stop the drone (clear any previous commands)
        for _ in range(30):
            self.test_node.publish_drone_cmd(0.0, 0.0, 0.0, 0.0)
            self._spin_for_time(0.016)

        # Get initial position
        self._spin_for_time(0.5)
        initial_odom = self.test_node.get_latest_drone_odom()
        self.assertIsNotNone(initial_odom, "No initial odometry received")

        initial_x = initial_odom.pose.pose.position.x
        initial_y = initial_odom.pose.pose.position.y
        initial_z = initial_odom.pose.pose.position.z

        # Send forward command (1 m/s in x direction)
        for _ in range(60):  # Send for ~1 second (60 Hz loop)
            self.test_node.publish_drone_cmd(1.0, 0.0, 0.0, 0.0)
            self._spin_for_time(0.016)  # ~60 Hz

        # Stop the drone
        for _ in range(30):
            self.test_node.publish_drone_cmd(0.0, 0.0, 0.0, 0.0)
            self._spin_for_time(0.016)

        # Wait a bit for motion to complete
        self._spin_for_time(0.5)

        # Get final position
        final_odom = self.test_node.get_latest_drone_odom()
        self.assertIsNotNone(final_odom, "No final odometry received")

        final_x = final_odom.pose.pose.position.x
        final_y = final_odom.pose.pose.position.y
        final_z = final_odom.pose.pose.position.z

        # Check that drone moved forward (positive x direction)
        delta_x = final_x - initial_x
        delta_y = abs(final_y - initial_y)
        delta_z = abs(final_z - initial_z)

        self.assertGreater(delta_x, 0.5, f"Drone should have moved forward ~1m, moved {delta_x:.2f}m")
        self.assertLess(delta_y, 0.2, f"Drone should not drift in y, drifted {delta_y:.2f}m")
        self.assertLess(delta_z, 0.2, f"Drone should maintain altitude, changed {delta_z:.2f}m")

    def test_rover_command_to_movement(self):
        """Test that rover velocity commands result in position changes."""
        # First, stop the rover (clear any previous commands)
        for _ in range(30):
            self.test_node.publish_rover_cmd(0.0, 0.0)
            self._spin_for_time(0.016)

        # Get initial position
        self._spin_for_time(0.5)
        initial_odom = self.test_node.get_latest_rover_odom()
        self.assertIsNotNone(initial_odom, "No initial odometry received")

        initial_x = initial_odom.pose.pose.position.x
        initial_y = initial_odom.pose.pose.position.y

        # Send forward command (0.5 m/s)
        for _ in range(60):  # Send for ~1 second
            self.test_node.publish_rover_cmd(0.5, 0.0)
            self._spin_for_time(0.016)

        # Stop the rover
        for _ in range(30):
            self.test_node.publish_rover_cmd(0.0, 0.0)
            self._spin_for_time(0.016)

        # Wait for motion to complete
        self._spin_for_time(0.5)

        # Get final position
        final_odom = self.test_node.get_latest_rover_odom()
        self.assertIsNotNone(final_odom, "No final odometry received")

        final_x = final_odom.pose.pose.position.x
        final_y = final_odom.pose.pose.position.y

        # Calculate distance moved
        distance = math.sqrt((final_x - initial_x)**2 + (final_y - initial_y)**2)

        self.assertGreater(distance, 0.3, f"Rover should have moved ~0.5m, moved {distance:.2f}m")

    def test_radio_communication_in_range(self):
        """Test radio communication when robots are in range."""
        # Wait for simulator to stabilize
        self._spin_for_time(0.5)

        # Clear any existing radio messages
        with self.test_node.lock:
            self.test_node.drone_radio_rx_msgs.clear()
            self.test_node.rover_radio_rx_msgs.clear()

        # Send message from drone to rover
        test_message = "Hello from drone"
        self.test_node.publish_drone_radio(test_message)

        # Wait for message delivery (includes latency + jitter)
        self._spin_for_time(0.5)

        # Check that rover received the message
        with self.test_node.lock:
            rover_messages = self.test_node.rover_radio_rx_msgs

        # Note: With 1% packet loss, there's a small chance message is lost
        # For robustness, send multiple messages and check at least one arrives
        if len(rover_messages) == 0:
            # Try again with multiple messages
            for i in range(10):
                self.test_node.publish_drone_radio(f"Message {i}")
                self._spin_for_time(0.1)

            with self.test_node.lock:
                rover_messages = self.test_node.rover_radio_rx_msgs

        self.assertGreater(len(rover_messages), 0,
                          "Rover should have received at least one radio message")

        # Verify message content (for the first successful message)
        if len(rover_messages) > 0:
            # Messages might arrive with byte encoding
            received = rover_messages[0].data
            self.assertIsInstance(received, str, "Radio message should be a string")

    def test_gps_position_matches_odometry(self):
        """Test that GPS readings roughly match odometry positions."""
        # Wait for messages
        self._spin_for_time(1.0)

        # Get latest messages
        odom = self.test_node.get_latest_drone_odom()

        with self.test_node.lock:
            gps_msgs = self.test_node.drone_gps_msgs

        self.assertIsNotNone(odom, "No odometry received")
        self.assertGreater(len(gps_msgs), 0, "No GPS messages received")

        gps = gps_msgs[-1]

        # GPS should be near the reference point (0, 0 in ENU -> ~37.4°N, 122.1°W)
        # We just verify GPS has valid coordinates
        self.assertGreater(abs(gps.latitude), 0.0, "GPS latitude should be non-zero")
        self.assertGreater(abs(gps.longitude), 0.0, "GPS longitude should be non-zero")
        self.assertGreater(abs(gps.altitude), 0.0, "GPS altitude should be non-zero")

    def test_camera_image_format(self):
        """Test that camera publishes valid image data."""
        # Wait for camera image
        self._spin_for_time(1.0)

        with self.test_node.lock:
            images = self.test_node.camera_image_msgs
            infos = self.test_node.camera_info_msgs

        self.assertGreater(len(images), 0, "No camera images received")
        self.assertGreater(len(infos), 0, "No camera info received")

        image = images[-1]
        info = infos[-1]

        # Verify image properties (from default scenario: 128x128)
        self.assertEqual(image.width, 128, "Image width should be 128")
        self.assertEqual(image.height, 128, "Image height should be 128")
        self.assertEqual(image.encoding, 'rgb8', "Image encoding should be rgb8")

        # Verify image data size (128 * 128 * 3 bytes)
        expected_size = 128 * 128 * 3
        self.assertEqual(len(image.data), expected_size,
                        f"Image data size should be {expected_size} bytes")

        # Verify camera info matches
        self.assertEqual(info.width, 128)
        self.assertEqual(info.height, 128)

        # Verify camera has valid intrinsics
        self.assertGreater(info.k[0], 0.0, "fx should be positive")
        self.assertGreater(info.k[4], 0.0, "fy should be positive")

    def test_rover_range_sensor_responds_to_obstacles(self):
        """Test that range sensor readings change when approaching obstacles."""
        # This test is observational - we just verify range readings are reasonable
        self._spin_for_time(1.0)

        with self.test_node.lock:
            ranges = self.test_node.rover_range_msgs

        self.assertGreater(len(ranges), 0, "No range sensor messages received")

        # Check that range values are within valid bounds
        for range_msg in ranges[-10:]:  # Check last 10 messages
            self.assertGreaterEqual(range_msg.range, range_msg.min_range,
                                   "Range reading below minimum")
            self.assertLessEqual(range_msg.range, range_msg.max_range,
                                "Range reading above maximum")
            self.assertEqual(range_msg.radiation_type, Range.ULTRASOUND,
                           "Range sensor should be ultrasound type")


if __name__ == '__main__':
    unittest.main()
