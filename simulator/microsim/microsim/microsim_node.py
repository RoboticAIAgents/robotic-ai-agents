#!/usr/bin/env python3
"""
MicroSim Node - Main ROS 2 node for the simulator.

Responsibilities:
- Initialize all subsystems (world, physics, sensors, etc.)
- Run main simulation loop at 60 Hz
- Publish sensor data and TF transforms
- Handle services (reset, pause, step, seed)
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import NavSatFix, Range, Image, CameraInfo
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, Empty
from std_msgs.msg import Header

from microsim.timekeeper import Timekeeper
from microsim.world import World
from microsim.physics import DronePhysics, RoverPhysics
from microsim.sensors import GPS, RangeSensor
from microsim.camera import PinholeCamera
from microsim.radio import RadioLink
from microsim.tf_broadcaster import TFBroadcaster


class MicroSimNode(Node):
    """Main simulator node."""

    def __init__(self):
        super().__init__('microsim')

        self.get_logger().info('MicroSim node starting...')

        # Initialize subsystems
        self.timekeeper = Timekeeper(dt=1.0/60.0)
        self.world = World(size=(100.0, 100.0), resolution=0.5)
        self.drone = DronePhysics(max_velocity=3.0)
        self.rover = RoverPhysics(max_velocity=1.0)

        # Sensors
        self.drone_gps = GPS(noise_std=0.1, rate_hz=10.0)
        self.rover_gps = GPS(noise_std=0.1, rate_hz=10.0)
        self.rover_range = RangeSensor(max_range=10.0, rate_hz=20.0)
        self.drone_camera = PinholeCamera(width=128, height=128, fov_deg=90.0, rate_hz=2.0)

        # Radio link
        self.radio = RadioLink(max_range=100.0)

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

    def publish_odometry(self, timestamp):
        """Publish odometry for both robots."""
        # Drone odometry
        drone_odom = Odometry()
        drone_odom.header.stamp = timestamp
        drone_odom.header.frame_id = 'world'
        drone_odom.child_frame_id = 'drone/base_link'
        drone_odom.pose.pose.position.x = self.drone.state.x
        drone_odom.pose.pose.position.y = self.drone.state.y
        drone_odom.pose.pose.position.z = self.drone.state.z
        self.drone_odom_pub.publish(drone_odom)

        # Rover odometry
        rover_odom = Odometry()
        rover_odom.header.stamp = timestamp
        rover_odom.header.frame_id = 'world'
        rover_odom.child_frame_id = 'rover/base_link'
        rover_odom.pose.pose.position.x = self.rover.state.x
        rover_odom.pose.pose.position.y = self.rover.state.y
        rover_odom.pose.pose.position.z = 0.0
        self.rover_odom_pub.publish(rover_odom)

    def update_sensors(self, dt: float, sim_time: float, timestamp):
        """Update and publish sensor data."""
        # Drone GPS
        gps_reading = self.drone_gps.update(dt, self.drone.state.x,
                                            self.drone.state.y, self.drone.state.z)
        if gps_reading is not None:
            # TODO: Convert to proper NavSatFix with lat/lon
            # For now, publish placeholder
            pass

        # Rover GPS
        gps_reading = self.rover_gps.update(dt, self.rover.state.x,
                                            self.rover.state.y, 0.0)
        if gps_reading is not None:
            # TODO: Convert to proper NavSatFix
            pass

        # Rover range sensor
        # TODO: Compute actual range by raycasting
        true_range = 10.0  # Placeholder
        range_reading = self.rover_range.update(dt, true_range)
        if range_reading is not None:
            # TODO: Publish proper Range message
            pass

        # Drone camera
        if self.drone_camera.update(dt):
            # TODO: Render and publish image + camera_info
            pass

    def reset_callback(self, request, response):
        """Handle reset service."""
        self.get_logger().info('Reset requested')
        self.timekeeper.reset()
        self.drone.reset()
        self.rover.reset()
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
