#!/usr/bin/env python3
"""
Reference Autonomous Drone Controller for MicroSim

This is a simple "dumb" controller that demonstrates the structure for
building intelligent AI-based drone controllers. It follows a set of
waypoints using basic proportional control.

Use this as a template for your AI agent implementation:
- Replace the simple waypoint logic with AI decision-making
- Add perception processing (camera, sensors)
- Implement planning and reasoning algorithms
- Integrate with LLM/AI agents for high-level control

Architecture:
1. Sensor callbacks update internal state
2. Control loop runs at fixed rate (10 Hz)
3. Decision logic computes desired actions
4. Velocity commands are published

Author: Reference implementation for AI agent development
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, NavSatFix
from std_srvs.srv import Empty
import numpy as np
import math


class AutonomousDroneController(Node):
    """
    Simple autonomous drone controller template.

    This controller demonstrates the basic structure for AI-based control:
    - Subscribes to all relevant sensor topics
    - Maintains internal state representation
    - Runs decision loop at fixed rate
    - Publishes velocity commands

    To add AI intelligence, replace the simple_decision_logic() method
    with your AI agent's decision-making process.
    """

    def __init__(self):
        super().__init__('autonomous_drone_controller')

        # ====================================================================
        # CONTROL PARAMETERS
        # ====================================================================
        # These can be tuned or replaced with AI-learned parameters

        self.control_rate_hz = 10.0  # How often to compute control commands
        self.waypoint_tolerance = 3.0  # meters - generous tolerance to account for oscillations
        self.max_speed = 1.5  # m/s - reduced for more stable control

        # PD controller gains (Proportional + Derivative for damping)
        # NOTE: High Kd (damping) is critical to prevent oscillations
        self.position_kp = 0.4  # Proportional gain for XY position (reduced)
        self.position_kd = 1.2  # Derivative gain for XY damping (INCREASED - strong damping)
        self.altitude_kp = 0.6  # Proportional gain for altitude (reduced)
        self.altitude_kd = 1.5  # Derivative gain for altitude damping (INCREASED)

        # ====================================================================
        # INTERNAL STATE
        # ====================================================================
        # This represents the drone's current knowledge about itself and the world

        # Pose state (from odometry - ground truth)
        self.position = np.array([0.0, 0.0, 0.0])  # [x, y, z] in world frame
        self.velocity = np.array([0.0, 0.0, 0.0])  # [vx, vy, vz]
        self.yaw = 0.0  # radians

        # GPS state (noisy sensor)
        self.gps_position = None  # [lat, lon, alt] - if you want to use GPS

        # Camera state
        self.latest_camera_image = None  # Most recent camera frame
        self.camera_timestamp = None

        # Mission state
        self.current_waypoint_index = 0
        self.mission_active = False

        # ====================================================================
        # MISSION DEFINITION
        # ====================================================================
        # Define waypoints for the drone to visit
        # Format: [x, y, z, yaw] in world frame
        #
        # REPLACE THIS with AI-generated waypoints or goals

        self.waypoints = [
            [5.0, 5.0, 15.0, 0.0],      # Move to (5, 5), maintain altitude
            [10.0, 5.0, 15.0, 1.57],    # Move to (10, 5), turn right
            [10.0, 10.0, 30.0, 3.14],   # Move to (10, 10), go higher to 30m
            [0.0, 10.0, 30.0, -1.57],   # Move to (0, 10), maintain 30m
            [0.0, 0.0, 15.0, 0.0],      # Return to start, descend to 15m
            [0.0, 0.0, 1.0, 0.0],       # Final descent to 1m for landing
        ]

        # ====================================================================
        # ROS 2 PUBLISHERS
        # ====================================================================

        self.cmd_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)

        # ====================================================================
        # ROS 2 SUBSCRIBERS
        # ====================================================================
        # Subscribe to all available sensor data

        # Ground truth odometry (high rate, accurate)
        self.odom_sub = self.create_subscription(
            Odometry, '/drone/odom', self.odometry_callback, 10)

        # GPS sensor (lower rate, noisy)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/drone/gps', self.gps_callback, 10)

        # Camera feed
        self.camera_sub = self.create_subscription(
            Image, '/drone/camera/image_raw', self.camera_callback, 10)

        # ====================================================================
        # ROS 2 SERVICE CLIENTS
        # ====================================================================

        self.reset_client = self.create_client(Empty, '/sim/reset')

        # ====================================================================
        # CONTROL LOOP TIMER
        # ====================================================================
        # This is where the "thinking" happens

        control_period = 1.0 / self.control_rate_hz
        self.control_timer = self.create_timer(control_period, self.control_loop)

        self.get_logger().info('Autonomous Drone Controller initialized!')
        self.get_logger().info(f'Mission has {len(self.waypoints)} waypoints')
        self.get_logger().info('Call start_mission() to begin autonomous flight')

    # ========================================================================
    # SENSOR CALLBACKS
    # ========================================================================
    # These update the internal state based on sensor data

    def odometry_callback(self, msg: Odometry):
        """Update position and velocity from odometry."""
        self.position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        self.velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def gps_callback(self, msg: NavSatFix):
        """Update GPS position (noisy sensor)."""
        self.gps_position = np.array([
            msg.latitude,
            msg.longitude,
            msg.altitude
        ])
        # Note: You may want to convert lat/lon to local XY coordinates
        # for easier use in control algorithms

    def camera_callback(self, msg: Image):
        """Store latest camera image for perception processing."""
        self.latest_camera_image = msg
        self.camera_timestamp = msg.header.stamp

        # ================================================================
        # AI PERCEPTION HOOK
        # ================================================================
        # This is where you would process the camera image with AI:
        # - Object detection (obstacles, targets, landmarks)
        # - Semantic segmentation
        # - Visual odometry
        # - Scene understanding for decision-making
        #
        # Example:
        #   obstacles = self.detect_obstacles(msg)
        #   self.update_world_model(obstacles)

    # ========================================================================
    # CONTROL LOOP - THE "BRAIN"
    # ========================================================================

    def control_loop(self):
        """
        Main control loop - runs at fixed rate.

        This is where the intelligence goes. Replace the simple waypoint
        logic with your AI agent's decision-making:

        1. Perceive: Process sensor data
        2. Think: AI reasoning/planning (LLM, RL, planning algorithms)
        3. Act: Convert decisions to velocity commands
        """

        if not self.mission_active:
            # Mission not started, hover in place
            self.publish_velocity(0, 0, 0, 0)
            return

        # Check if mission is complete
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('Mission complete!', throttle_duration_sec=5.0)
            self.publish_velocity(0, 0, 0, 0)
            return

        # ================================================================
        # STEP 1: PERCEPTION
        # ================================================================
        # Process sensors to understand the current situation

        current_pos = self.position
        current_waypoint = self.waypoints[self.current_waypoint_index]
        target_pos = np.array(current_waypoint[:3])
        target_yaw = current_waypoint[3]

        # ================================================================
        # STEP 2: DECISION MAKING
        # ================================================================
        # Decide what action to take
        #
        # REPLACE THIS with your AI agent logic:
        # - LLM-based high-level planning
        # - Reinforcement learning policy
        # - Classical planning algorithms
        # - Hybrid AI approaches

        velocity_command = self.simple_decision_logic(
            current_pos, target_pos, target_yaw
        )

        # ================================================================
        # STEP 3: ACTION
        # ================================================================
        # Execute the decision

        self.publish_velocity(*velocity_command)

        # Check if waypoint reached
        distance_to_waypoint = np.linalg.norm(current_pos - target_pos)

        # Log progress every 2 seconds
        self.get_logger().info(
            f'WP{self.current_waypoint_index}: dist={distance_to_waypoint:.2f}m, '
            f'pos=[{current_pos[0]:.1f}, {current_pos[1]:.1f}, {current_pos[2]:.1f}], '
            f'cmd=[{velocity_command[0]:.2f}, {velocity_command[1]:.2f}, {velocity_command[2]:.2f}]',
            throttle_duration_sec=2.0
        )

        if distance_to_waypoint < self.waypoint_tolerance:
            self.get_logger().info(
                f'✓ Reached waypoint {self.current_waypoint_index}: {target_pos}'
            )
            self.current_waypoint_index += 1

    # ========================================================================
    # DECISION LOGIC (REPLACE THIS WITH AI)
    # ========================================================================

    def simple_decision_logic(self, current_pos, target_pos, target_yaw):
        """
        PD controller for stable waypoint following.

        Uses Proportional-Derivative control:
        - Proportional term: drives toward goal
        - Derivative term: provides damping to prevent oscillations

        REPLACE THIS METHOD with your AI agent's decision-making:

        Example AI integration points:
        1. LLM-based planning:
           action = llm_agent.decide(current_state, goal, obstacles)

        2. Reinforcement Learning:
           action = rl_policy.get_action(observation)

        3. Behavior Trees:
           action = behavior_tree.tick(world_state)

        4. Classical Planning:
           path = planner.plan(current_pos, goal, obstacles)
           action = path_follower.get_velocity(path, current_pos)

        Returns:
            tuple: (vx, vy, vz, omega_z) - velocity command
        """

        # Compute position error (proportional term)
        error = target_pos - current_pos

        # PD control for XY position
        # P term: position error, D term: current velocity (acts as damping)
        vx_desired = self.position_kp * error[0] - self.position_kd * self.velocity[0]
        vy_desired = self.position_kp * error[1] - self.position_kd * self.velocity[1]

        # PD control for altitude
        vz_desired = self.altitude_kp * error[2] - self.altitude_kd * self.velocity[2]

        # Yaw control (simple proportional - rotation doesn't oscillate as much)
        yaw_error = self.normalize_angle(target_yaw - self.yaw)
        omega_z = 0.8 * yaw_error

        # Clamp velocities to max speed
        vx = np.clip(vx_desired, -self.max_speed, self.max_speed)
        vy = np.clip(vy_desired, -self.max_speed, self.max_speed)
        vz = np.clip(vz_desired, -self.max_speed, self.max_speed)
        omega_z = np.clip(omega_z, -1.0, 1.0)

        return (vx, vy, vz, omega_z)

    # ========================================================================
    # UTILITY METHODS
    # ========================================================================

    def publish_velocity(self, vx, vy, vz, omega_z):
        """Publish velocity command to the drone."""
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.linear.z = float(vz)
        cmd.angular.z = float(omega_z)
        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # ========================================================================
    # MISSION CONTROL
    # ========================================================================

    def start_mission(self):
        """Start autonomous mission execution."""
        self.mission_active = True
        self.current_waypoint_index = 0
        self.get_logger().info('Mission started!')

    def stop_mission(self):
        """Stop mission and hover."""
        self.mission_active = False
        self.publish_velocity(0, 0, 0, 0)
        self.get_logger().info('Mission stopped!')

    def reset_mission(self):
        """Reset mission to beginning."""
        self.current_waypoint_index = 0
        self.get_logger().info('Mission reset!')

    def set_waypoints(self, waypoints):
        """
        Set new waypoints for the mission.

        Args:
            waypoints: List of [x, y, z, yaw] waypoints
        """
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.get_logger().info(f'Updated mission with {len(waypoints)} waypoints')


def main(args=None):
    rclpy.init(args=args)

    controller = AutonomousDroneController()

    # Auto-start the mission
    # Comment this out if you want manual control of when to start
    controller.start_mission()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop_mission()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
