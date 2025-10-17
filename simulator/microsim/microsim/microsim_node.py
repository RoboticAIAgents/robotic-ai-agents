#!/usr/bin/env python3
"""
MicroSim Node - Main ROS 2 node for the simulator.

Responsibilities:
- Load scenario configuration from YAML
- Initialize all subsystems (world, physics, sensors, etc.)
- Run main simulation loop at 60 Hz
- Publish sensor data and TF transforms
- Handle services (reset, pause, step, seed)
"""

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import NavSatFix, Range, Image, CameraInfo, NavSatStatus
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, Empty
from std_msgs.msg import Header, String
import numpy as np

from microsim.timekeeper import Timekeeper
from microsim.world import World
from microsim.physics import DronePhysics, RoverPhysics
from microsim.sensors import GPS, RangeSensor
from microsim.camera import PinholeCamera
from microsim.radio import RadioLink
from microsim.tf_broadcaster import TFBroadcaster
from microsim.scenario_loader import ScenarioLoader
from microsim.utils import enu_to_wgs84, euler_to_quaternion


class MicroSimNode(Node):
    """Main simulator node."""

    def __init__(self, scenario_file: str = None):
        super().__init__('microsim')

        self.get_logger().info('MicroSim node starting...')

        # Load scenario configuration
        if scenario_file is None:
            # Use default scenario
            pkg_dir = os.path.dirname(os.path.dirname(__file__))
            scenario_file = os.path.join(pkg_dir, 'scenarios', 'default.yaml')

        self.get_logger().info(f'Loading scenario from: {scenario_file}')
        self.scenario = ScenarioLoader.load_from_file(scenario_file)

        # Initialize world with scenario configuration
        self.world = World(
            size=tuple(self.scenario.world_size),
            resolution=self.scenario.world_resolution
        )

        # Load obstacles from scenario
        for feature in self.scenario.features:
            semantic = ScenarioLoader.feature_type_to_semantic(feature.type)
            self.world.set_region(
                feature.position[0],
                feature.position[1],
                feature.radius,
                feature.height,
                semantic
            )

        self.get_logger().info(f'Loaded {len(self.scenario.features)} world features')

        # Initialize timekeeper
        self.timekeeper = Timekeeper(dt=1.0/60.0)

        # Initialize robots with scenario parameters
        self.drone = DronePhysics(
            max_velocity=self.scenario.drone_max_velocity,
            max_accel=self.scenario.drone_max_accel
        )
        self.drone.reset(
            x=self.scenario.drone_pose.x,
            y=self.scenario.drone_pose.y,
            z=self.scenario.drone_pose.z
        )
        self.drone.state.yaw = self.scenario.drone_pose.yaw

        self.rover = RoverPhysics(
            max_velocity=self.scenario.rover_max_velocity,
            max_omega=self.scenario.rover_max_omega
        )
        self.rover.reset(
            x=self.scenario.rover_pose.x,
            y=self.scenario.rover_pose.y,
            theta=self.scenario.rover_pose.theta
        )

        # Initialize sensors with scenario parameters
        self.drone_gps = GPS(
            noise_std=self.scenario.gps_noise_std,
            rate_hz=self.scenario.gps_rate_hz
        )
        self.rover_gps = GPS(
            noise_std=self.scenario.gps_noise_std,
            rate_hz=self.scenario.gps_rate_hz
        )
        self.rover_range = RangeSensor(
            max_range=self.scenario.range_max_range,
            noise_std=self.scenario.range_noise_std,
            rate_hz=self.scenario.range_rate_hz
        )
        self.drone_camera = PinholeCamera(
            width=self.scenario.camera_width,
            height=self.scenario.camera_height,
            fov_deg=self.scenario.camera_fov_deg,
            rate_hz=self.scenario.camera_rate_hz
        )

        # Radio link
        self.radio = RadioLink(
            max_range=self.scenario.radio_max_range,
            base_latency=self.scenario.radio_base_latency,
            jitter=self.scenario.radio_jitter,
            packet_loss=self.scenario.radio_packet_loss
        )

        # TF broadcaster
        self.tf_broadcaster = TFBroadcaster(self)

        # Subscribers for velocity commands
        self.create_subscription(Twist, '/drone/cmd_vel', self.drone_cmd_vel_callback, 10)
        self.create_subscription(Twist, '/rover/cmd_vel', self.rover_cmd_vel_callback, 10)

        # Publishers for odometry
        self.drone_odom_pub = self.create_publisher(Odometry, '/drone/odom', 10)
        self.rover_odom_pub = self.create_publisher(Odometry, '/rover/odom', 10)

        # Publishers for GPS
        self.drone_gps_pub = self.create_publisher(NavSatFix, '/drone/gps', 10)
        self.rover_gps_pub = self.create_publisher(NavSatFix, '/rover/gps', 10)

        # Publishers for range sensor
        self.rover_range_pub = self.create_publisher(Range, '/rover/range', 10)

        # Publishers for camera
        self.camera_image_pub = self.create_publisher(Image, '/drone/camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/drone/camera/camera_info', 10)

        # Radio communication publishers (best effort QoS for lossy channel simulation)
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        radio_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.drone_radio_rx_pub = self.create_publisher(String, '/radio/drone_rx', radio_qos)
        self.rover_radio_rx_pub = self.create_publisher(String, '/radio/rover_rx', radio_qos)

        # Radio communication subscribers
        self.create_subscription(String, '/radio/drone_tx', self.drone_radio_tx_callback, radio_qos)
        self.create_subscription(String, '/radio/rover_tx', self.rover_radio_tx_callback, radio_qos)

        # Services
        self.create_service(Empty, '~/reset', self.reset_callback)
        self.create_service(Trigger, '~/pause', self.pause_callback)

        # Command velocity storage
        self.drone_cmd = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'vyaw': 0.0}
        self.rover_cmd = {'v': 0.0, 'omega': 0.0}

        # Control flags
        self.paused = False

        # Main simulation timer (60 Hz)
        self.timer = self.create_timer(1.0/60.0, self.simulation_step)

        self.get_logger().info('MicroSim node initialized successfully')

    def drone_cmd_vel_callback(self, msg: Twist):
        """Handle drone velocity commands."""
        self.drone_cmd['vx'] = msg.linear.x
        self.drone_cmd['vy'] = msg.linear.y
        self.drone_cmd['vz'] = msg.linear.z
        self.drone_cmd['vyaw'] = msg.angular.z

    def rover_cmd_vel_callback(self, msg: Twist):
        """Handle rover velocity commands."""
        self.rover_cmd['v'] = msg.linear.x
        self.rover_cmd['omega'] = msg.angular.z

    def drone_radio_tx_callback(self, msg: String):
        """Handle drone transmitting radio message."""
        # Calculate distance between robots
        dx = self.drone.state.x - self.rover.state.x
        dy = self.drone.state.y - self.rover.state.y
        distance = np.sqrt(dx*dx + dy*dy)

        # Send message through radio link
        success = self.radio.send_message(
            from_robot='drone',
            to_robot='rover',
            payload=msg.data.encode('utf-8'),
            current_time=self.timekeeper.get_time(),
            distance=distance
        )

        if not success:
            self.get_logger().debug(
                f'Radio TX failed: distance={distance:.1f}m, '
                f'max_range={self.scenario.radio_max_range}m'
            )

    def rover_radio_tx_callback(self, msg: String):
        """Handle rover transmitting radio message."""
        # Calculate distance between robots
        dx = self.drone.state.x - self.rover.state.x
        dy = self.drone.state.y - self.rover.state.y
        distance = np.sqrt(dx*dx + dy*dy)

        # Send message through radio link
        success = self.radio.send_message(
            from_robot='rover',
            to_robot='drone',
            payload=msg.data.encode('utf-8'),
            current_time=self.timekeeper.get_time(),
            distance=distance
        )

        if not success:
            self.get_logger().debug(
                f'Radio TX failed: distance={distance:.1f}m, '
                f'max_range={self.scenario.radio_max_range}m'
            )

    def simulation_step(self):
        """Main simulation step callback (60 Hz)."""
        if self.paused:
            return

        # Advance time
        dt = self.timekeeper.dt
        sim_time = self.timekeeper.step()

        # Update physics
        self.drone.update(dt, **self.drone_cmd)
        self.rover.update(dt, **self.rover_cmd)

        # Get current ROS time
        timestamp = self.get_clock().now().to_msg()

        # Publish TF transforms
        self.tf_broadcaster.publish_drone_tf(
            timestamp,
            self.drone.state.x, self.drone.state.y, self.drone.state.z,
            self.drone.state.roll, self.drone.state.pitch, self.drone.state.yaw
        )
        self.tf_broadcaster.publish_rover_tf(
            timestamp,
            self.rover.state.x, self.rover.state.y, self.rover.state.theta
        )
        self.tf_broadcaster.publish_camera_tf(timestamp)

        # Publish odometry (every step)
        self.publish_odometry(timestamp)

        # Update and publish sensors (at their respective rates)
        self.update_sensors(dt, sim_time, timestamp)

        # Process radio message delivery
        self.update_radio(sim_time, timestamp)

    def publish_odometry(self, timestamp):
        """Publish odometry for both robots with proper quaternions."""
        # Drone odometry
        drone_odom = Odometry()
        drone_odom.header.stamp = timestamp
        drone_odom.header.frame_id = 'world'
        drone_odom.child_frame_id = 'drone/base_link'

        # Position
        drone_odom.pose.pose.position.x = self.drone.state.x
        drone_odom.pose.pose.position.y = self.drone.state.y
        drone_odom.pose.pose.position.z = self.drone.state.z

        # Orientation (convert Euler to quaternion)
        qx, qy, qz, qw = euler_to_quaternion(
            self.drone.state.roll,
            self.drone.state.pitch,
            self.drone.state.yaw
        )
        drone_odom.pose.pose.orientation.x = qx
        drone_odom.pose.pose.orientation.y = qy
        drone_odom.pose.pose.orientation.z = qz
        drone_odom.pose.pose.orientation.w = qw

        # Velocity (in body frame)
        drone_odom.twist.twist.linear.x = self.drone.state.vx
        drone_odom.twist.twist.linear.y = self.drone.state.vy
        drone_odom.twist.twist.linear.z = self.drone.state.vz
        drone_odom.twist.twist.angular.z = self.drone.state.vyaw

        self.drone_odom_pub.publish(drone_odom)

        # Rover odometry
        rover_odom = Odometry()
        rover_odom.header.stamp = timestamp
        rover_odom.header.frame_id = 'world'
        rover_odom.child_frame_id = 'rover/base_link'

        # Position
        rover_odom.pose.pose.position.x = self.rover.state.x
        rover_odom.pose.pose.position.y = self.rover.state.y
        rover_odom.pose.pose.position.z = 0.0

        # Orientation (2D rotation)
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, self.rover.state.theta)
        rover_odom.pose.pose.orientation.x = qx
        rover_odom.pose.pose.orientation.y = qy
        rover_odom.pose.pose.orientation.z = qz
        rover_odom.pose.pose.orientation.w = qw

        # Velocity
        rover_odom.twist.twist.linear.x = self.rover.state.v
        rover_odom.twist.twist.angular.z = self.rover.state.omega

        self.rover_odom_pub.publish(rover_odom)

    def update_sensors(self, dt: float, sim_time: float, timestamp):
        """Update and publish sensor data."""

        # Drone GPS
        gps_reading = self.drone_gps.update(
            dt,
            self.drone.state.x,
            self.drone.state.y,
            self.drone.state.z
        )
        if gps_reading is not None:
            x, y, z = gps_reading
            lat, lon, alt = enu_to_wgs84(x, y, z)

            gps_msg = NavSatFix()
            gps_msg.header.stamp = timestamp
            gps_msg.header.frame_id = 'drone/gps_link'
            gps_msg.status.status = NavSatStatus.STATUS_FIX
            gps_msg.status.service = NavSatStatus.SERVICE_GPS
            gps_msg.latitude = lat
            gps_msg.longitude = lon
            gps_msg.altitude = alt

            # Covariance (diagonal with noise_std^2)
            cov = self.scenario.gps_noise_std ** 2
            gps_msg.position_covariance = [
                cov, 0.0, 0.0,
                0.0, cov, 0.0,
                0.0, 0.0, cov
            ]
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            self.drone_gps_pub.publish(gps_msg)

        # Rover GPS
        gps_reading = self.rover_gps.update(
            dt,
            self.rover.state.x,
            self.rover.state.y,
            0.0
        )
        if gps_reading is not None:
            x, y, z = gps_reading
            lat, lon, alt = enu_to_wgs84(x, y, z)

            gps_msg = NavSatFix()
            gps_msg.header.stamp = timestamp
            gps_msg.header.frame_id = 'rover/gps_link'
            gps_msg.status.status = NavSatStatus.STATUS_FIX
            gps_msg.status.service = NavSatStatus.SERVICE_GPS
            gps_msg.latitude = lat
            gps_msg.longitude = lon
            gps_msg.altitude = alt

            cov = self.scenario.gps_noise_std ** 2
            gps_msg.position_covariance = [
                cov, 0.0, 0.0,
                0.0, cov, 0.0,
                0.0, 0.0, cov
            ]
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            self.rover_gps_pub.publish(gps_msg)

        # Rover range sensor
        # Compute true range by raycasting
        true_range = self.compute_range(
            self.rover.state.x,
            self.rover.state.y,
            self.rover.state.theta
        )

        range_reading = self.rover_range.update(dt, true_range)
        if range_reading is not None:
            range_msg = Range()
            range_msg.header.stamp = timestamp
            range_msg.header.frame_id = 'rover/range_link'
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = 0.1  # 0.1 rad cone
            range_msg.min_range = 0.0
            range_msg.max_range = self.scenario.range_max_range
            range_msg.range = range_reading

            self.rover_range_pub.publish(range_msg)

        # Drone camera
        if self.drone_camera.update(dt):
            # Render image
            image_array = self.drone_camera.render(
                self.world,
                self.drone.state.x,
                self.drone.state.y,
                self.drone.state.z,
                self.drone.state.yaw,
                self.drone.state.pitch
            )

            # Publish Image message
            image_msg = Image()
            image_msg.header.stamp = timestamp
            image_msg.header.frame_id = 'drone/camera_link'
            image_msg.height = self.drone_camera.height
            image_msg.width = self.drone_camera.width
            image_msg.encoding = 'rgb8'
            image_msg.is_bigendian = 0
            image_msg.step = self.drone_camera.width * 3
            image_msg.data = image_array.tobytes()

            self.camera_image_pub.publish(image_msg)

            # Publish CameraInfo message
            camera_info = self.drone_camera.get_camera_info()

            info_msg = CameraInfo()
            info_msg.header.stamp = timestamp
            info_msg.header.frame_id = 'drone/camera_link'
            info_msg.width = camera_info['width']
            info_msg.height = camera_info['height']
            info_msg.distortion_model = camera_info['distortion_model']
            info_msg.d = camera_info['D']
            info_msg.k = camera_info['K']
            info_msg.p = camera_info['P']

            self.camera_info_pub.publish(info_msg)

    def compute_range(self, x: float, y: float, theta: float) -> float:
        """
        Compute range to nearest obstacle in forward direction.

        Args:
            x, y: Rover position
            theta: Rover heading

        Returns:
            Distance to nearest obstacle in meters
        """
        max_range = self.scenario.range_max_range
        resolution = self.world.resolution

        # Cast ray in forward direction
        for distance in np.arange(0.0, max_range, resolution):
            ray_x = x + distance * np.cos(theta)
            ray_y = y + distance * np.sin(theta)

            # Check if this point is inside an obstacle
            height = self.world.get_height(ray_x, ray_y)

            # If height > 0.5m, it's an obstacle (rover-height threshold)
            if height > 0.5:
                return distance

        # No obstacle found, return max range
        return max_range

    def update_radio(self, sim_time: float, timestamp):
        """Process radio message delivery."""
        # Check for messages ready to be delivered to drone
        drone_messages = self.radio.receive_messages('drone', sim_time)
        for payload in drone_messages:
            msg = String()
            msg.data = payload.decode('utf-8')
            self.drone_radio_rx_pub.publish(msg)

        # Check for messages ready to be delivered to rover
        rover_messages = self.radio.receive_messages('rover', sim_time)
        for payload in rover_messages:
            msg = String()
            msg.data = payload.decode('utf-8')
            self.rover_radio_rx_pub.publish(msg)

    def reset_callback(self, request, response):
        """Handle reset service."""
        self.get_logger().info('Reset requested')

        self.timekeeper.reset()

        # Reset robots to initial poses from scenario
        self.drone.reset(
            x=self.scenario.drone_pose.x,
            y=self.scenario.drone_pose.y,
            z=self.scenario.drone_pose.z
        )
        self.drone.state.yaw = self.scenario.drone_pose.yaw

        self.rover.reset(
            x=self.scenario.rover_pose.x,
            y=self.scenario.rover_pose.y,
            theta=self.scenario.rover_pose.theta
        )

        return response

    def pause_callback(self, request, response):
        """Handle pause/unpause service."""
        self.paused = not self.paused
        status = 'paused' if self.paused else 'unpaused'
        self.get_logger().info(f'Simulation {status}')
        response.success = True
        response.message = status
        return response


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    # TODO: Parse scenario file from command line args
    node = MicroSimNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
