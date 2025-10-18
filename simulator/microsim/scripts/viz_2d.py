#!/usr/bin/env python3
"""
2D visualization tool for MicroSim using matplotlib
More stable on macOS than RViz - no OpenGL required
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
from cv_bridge import CvBridge


class MicroSimViz2D(Node):
    def __init__(self):
        super().__init__('microsim_viz_2d')

        # Create figure with subplots
        self.fig, (self.ax_map, self.ax_camera) = plt.subplots(1, 2, figsize=(14, 7))
        self.fig.canvas.manager.set_window_title('MicroSim 2D Visualization')

        # Map view setup
        self.ax_map.set_xlim(-50, 50)
        self.ax_map.set_ylim(-50, 50)
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.set_xlabel('X (meters) - East')
        self.ax_map.set_ylabel('Y (meters) - North')
        self.ax_map.set_title('Top-Down View')

        # Camera view setup
        self.ax_camera.set_title('Drone Camera Feed (128x128)')
        self.ax_camera.axis('off')

        # Data storage
        self.drone_pos = [0, 0, 40]
        self.drone_yaw = 0
        self.rover_pos = [10, 0, 0]
        self.rover_yaw = 0
        self.world_features = []
        self.camera_image = None

        # Plot elements (will be created on first update)
        self.drone_marker = None
        self.rover_marker = None
        self.drone_trail, = self.ax_map.plot([], [], 'r-', alpha=0.3, linewidth=1, label='Drone trail')
        self.rover_trail, = self.ax_map.plot([], [], 'b-', alpha=0.3, linewidth=1, label='Rover trail')
        self.drone_trail_x = []
        self.drone_trail_y = []
        self.rover_trail_x = []
        self.rover_trail_y = []
        self.camera_img_plot = None
        self.feature_patches = []

        # Text display for altitude
        self.drone_alt_text = self.ax_map.text(0.02, 0.98, '', transform=self.ax_map.transAxes,
                                               verticalalignment='top', fontsize=10,
                                               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Subscribers
        self.drone_odom_sub = self.create_subscription(
            Odometry, '/drone/odom', self.drone_odom_callback, 10)
        self.rover_odom_sub = self.create_subscription(
            Odometry, '/rover/odom', self.rover_odom_callback, 10)
        self.markers_sub = self.create_subscription(
            MarkerArray, '/world/markers', self.markers_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/drone/camera/image_raw', self.camera_callback, 10)

        # Animation
        self.anim = FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)

        self.get_logger().info('2D Visualization started - Close window to exit')

    def drone_odom_callback(self, msg):
        """Update drone position from odometry"""
        self.drone_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.drone_yaw = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))

        # Add to trail (keep last 100 points)
        self.drone_trail_x.append(self.drone_pos[0])
        self.drone_trail_y.append(self.drone_pos[1])
        if len(self.drone_trail_x) > 100:
            self.drone_trail_x.pop(0)
            self.drone_trail_y.pop(0)

    def rover_odom_callback(self, msg):
        """Update rover position from odometry"""
        self.rover_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.rover_yaw = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))

        # Add to trail (keep last 100 points)
        self.rover_trail_x.append(self.rover_pos[0])
        self.rover_trail_y.append(self.rover_pos[1])
        if len(self.rover_trail_x) > 100:
            self.rover_trail_x.pop(0)
            self.rover_trail_y.pop(0)

    def markers_callback(self, msg):
        """Update world features from markers"""
        self.world_features = []
        for marker in msg.markers:
            if marker.ns in ['obstacle', 'hazard', 'target']:
                feature = {
                    'type': marker.ns,
                    'x': marker.pose.position.x,
                    'y': marker.pose.position.y,
                    'radius': marker.scale.x / 2.0,  # diameter to radius
                    'color': (marker.color.r, marker.color.g, marker.color.b)
                }
                self.world_features.append(feature)

    def camera_callback(self, msg):
        """Update camera image"""
        try:
            # Convert ROS Image to numpy array
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.camera_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def update_plot(self, frame):
        """Update the matplotlib plot"""
        # Clear previous patches
        for patch in self.feature_patches:
            patch.remove()
        self.feature_patches = []

        # Draw world features
        for feature in self.world_features:
            circle = patches.Circle(
                (feature['x'], feature['y']),
                feature['radius'],
                facecolor=feature['color'],
                edgecolor='black',
                alpha=0.6,
                label=feature['type'].capitalize()
            )
            self.ax_map.add_patch(circle)
            self.feature_patches.append(circle)

        # Draw drone
        if self.drone_marker:
            self.drone_marker.remove()
        drone_size = 0.3
        drone_arrow_len = 1.0
        self.drone_marker = patches.FancyArrow(
            self.drone_pos[0], self.drone_pos[1],
            drone_arrow_len * np.cos(self.drone_yaw),
            drone_arrow_len * np.sin(self.drone_yaw),
            width=drone_size, head_width=drone_size*2, head_length=drone_size*1.5,
            fc='red', ec='darkred', alpha=0.8, label='Drone'
        )
        self.ax_map.add_patch(self.drone_marker)

        # Draw rover
        if self.rover_marker:
            self.rover_marker.remove()
        rover_size = 0.2
        rover_arrow_len = 0.8
        self.rover_marker = patches.FancyArrow(
            self.rover_pos[0], self.rover_pos[1],
            rover_arrow_len * np.cos(self.rover_yaw),
            rover_arrow_len * np.sin(self.rover_yaw),
            width=rover_size, head_width=rover_size*2, head_length=rover_size*1.5,
            fc='blue', ec='darkblue', alpha=0.8, label='Rover'
        )
        self.ax_map.add_patch(self.rover_marker)

        # Update trails
        self.drone_trail.set_data(self.drone_trail_x, self.drone_trail_y)
        self.rover_trail.set_data(self.rover_trail_x, self.rover_trail_y)

        # Update altitude text
        self.drone_alt_text.set_text(
            f'Drone: ({self.drone_pos[0]:.1f}, {self.drone_pos[1]:.1f}, {self.drone_pos[2]:.1f}m)\n'
            f'Rover: ({self.rover_pos[0]:.1f}, {self.rover_pos[1]:.1f}, {self.rover_pos[2]:.1f}m)'
        )

        # Update camera view
        if self.camera_image is not None:
            if self.camera_img_plot is None:
                self.camera_img_plot = self.ax_camera.imshow(self.camera_image)
            else:
                self.camera_img_plot.set_data(self.camera_image)

        # Only show legend on first update
        if frame == 0:
            handles, labels = self.ax_map.get_legend_handles_labels()
            # Remove duplicates
            by_label = dict(zip(labels, handles))
            self.ax_map.legend(by_label.values(), by_label.keys(), loc='upper right')


def main(args=None):
    rclpy.init(args=args)

    try:
        viz = MicroSimViz2D()
        plt.show(block=False)

        # Spin in background while matplotlib runs
        while plt.fignum_exists(viz.fig.number):
            rclpy.spin_once(viz, timeout_sec=0.01)
            plt.pause(0.01)

        viz.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
