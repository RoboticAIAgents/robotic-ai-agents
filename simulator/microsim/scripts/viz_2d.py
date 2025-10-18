#!/usr/bin/env python3
"""
3D visualization tool for MicroSim using matplotlib
More stable on macOS than RViz - no OpenGL required

Configuration:
Edit the VIEW_SCALE parameters below to adjust the viewing volume.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3D
import numpy as np
from cv_bridge import CvBridge


# ============================================================================
# VISUALIZATION CONFIGURATION
# ============================================================================
# Adjust these values to change the viewing volume and scale
#
# Tips:
# - For low-altitude flights (0-10m): Try Z_LIMIT=10, X/Y_LIMIT=20
# - For high-altitude flights (0-50m): Try Z_LIMIT=50, X/Y_LIMIT=50
# - Smaller limits = robots appear larger and easier to see
# - Larger limits = more of the world visible, but robots appear smaller
#
# The 2D map shows the entire world from above.
# The 3D view can be zoomed in to focus on the robots.

# 2D Map view limits (X-Y plane, top-down)
MAP_X_LIMIT = 50  # meters (±X range, shows full world)
MAP_Y_LIMIT = 50  # meters (±Y range, shows full world)

# 3D Perspective view limits (focused on robots)
VIEW_3D_X_LIMIT = 30  # meters (±X range, zoom in for better visibility)
VIEW_3D_Y_LIMIT = 30  # meters (±Y range, zoom in for better visibility)
VIEW_3D_Z_LIMIT = 15  # meters (0 to Z_max, altitude range)

# 3D View angle (can also rotate interactively with mouse)
VIEW_3D_ELEVATION = 20  # degrees (viewing angle above horizontal, 0=side view, 90=top view)
VIEW_3D_AZIMUTH = 45    # degrees (viewing angle rotation, 0=view from +Y, 90=view from +X)

# ============================================================================


class MicroSimViz2D(Node):
    def __init__(self):
        super().__init__('microsim_viz_3d')

        # Create figure with subplots: 2x2 grid
        # Top-left: 2D map, Bottom-left: 3D view, Right: Camera (spans both rows)
        self.fig = plt.figure(figsize=(16, 9))
        gs = self.fig.add_gridspec(2, 2, width_ratios=[1, 1], height_ratios=[1, 1])

        self.ax_map = self.fig.add_subplot(gs[0, 0])  # Top-left: 2D map
        self.ax_3d = self.fig.add_subplot(gs[1, 0], projection='3d')  # Bottom-left: 3D view
        self.ax_camera = self.fig.add_subplot(gs[:, 1])  # Right: Camera (full height)

        self.fig.canvas.manager.set_window_title('MicroSim 3D Visualization')

        # Map view setup (2D top-down)
        self.ax_map.set_xlim(-MAP_X_LIMIT, MAP_X_LIMIT)
        self.ax_map.set_ylim(-MAP_Y_LIMIT, MAP_Y_LIMIT)
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.set_xlabel('X (meters) - East')
        self.ax_map.set_ylabel('Y (meters) - North')
        self.ax_map.set_title('Top-Down View (2D)')

        # 3D view setup
        self.ax_3d.set_xlim(-VIEW_3D_X_LIMIT, VIEW_3D_X_LIMIT)
        self.ax_3d.set_ylim(-VIEW_3D_Y_LIMIT, VIEW_3D_Y_LIMIT)
        self.ax_3d.set_zlim(0, VIEW_3D_Z_LIMIT)
        self.ax_3d.set_xlabel('X (meters) - East')
        self.ax_3d.set_ylabel('Y (meters) - North')
        self.ax_3d.set_zlabel('Z (meters) - Up')
        self.ax_3d.set_title('3D Perspective View')
        self.ax_3d.view_init(elev=VIEW_3D_ELEVATION, azim=VIEW_3D_AZIMUTH)

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

        # Plot elements for 2D view (will be created on first update)
        self.drone_marker = None
        self.rover_marker = None
        self.drone_trail, = self.ax_map.plot([], [], 'r-', alpha=0.3, linewidth=1, label='Drone trail')
        self.rover_trail, = self.ax_map.plot([], [], 'b-', alpha=0.3, linewidth=1, label='Rover trail')
        self.drone_trail_x = []
        self.drone_trail_y = []
        self.drone_trail_z = []
        self.rover_trail_x = []
        self.rover_trail_y = []
        self.rover_trail_z = []
        self.camera_img_plot = None
        self.feature_patches = []

        # Plot elements for 3D view
        self.drone_3d_quiver = None
        self.rover_3d_quiver = None
        self.drone_trail_3d = None
        self.rover_trail_3d = None
        self.feature_3d_artists = []

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

        self.get_logger().info('3D Visualization started - Close window to exit')

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
        self.drone_trail_z.append(self.drone_pos[2])
        if len(self.drone_trail_x) > 100:
            self.drone_trail_x.pop(0)
            self.drone_trail_y.pop(0)
            self.drone_trail_z.pop(0)

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
        self.rover_trail_z.append(self.rover_pos[2])
        if len(self.rover_trail_x) > 100:
            self.rover_trail_x.pop(0)
            self.rover_trail_y.pop(0)
            self.rover_trail_z.pop(0)

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

        # === 3D View Updates ===
        self.update_3d_view()

    def update_3d_view(self):
        """Update the 3D perspective view"""
        # Clear previous 3D artists
        for artist in self.feature_3d_artists:
            artist.remove()
        self.feature_3d_artists = []

        # Draw ground plane (simple grid)
        if not hasattr(self, 'ground_plane_drawn'):
            xx, yy = np.meshgrid([-VIEW_3D_X_LIMIT, VIEW_3D_X_LIMIT],
                                 [-VIEW_3D_Y_LIMIT, VIEW_3D_Y_LIMIT])
            zz = np.zeros_like(xx)
            self.ax_3d.plot_surface(xx, yy, zz, alpha=0.1, color='gray')
            self.ground_plane_drawn = True

        # Draw world features as 3D cylinders
        for feature in self.world_features:
            # Create cylinder
            radius = feature['radius']
            height = 5.0 if feature['type'] == 'obstacle' else 0.5  # Obstacles taller
            theta = np.linspace(0, 2*np.pi, 20)
            z_cyl = np.linspace(0, height, 10)
            theta_grid, z_grid = np.meshgrid(theta, z_cyl)
            x_cyl = feature['x'] + radius * np.cos(theta_grid)
            y_cyl = feature['y'] + radius * np.sin(theta_grid)

            surf = self.ax_3d.plot_surface(x_cyl, y_cyl, z_grid,
                                          color=feature['color'], alpha=0.6,
                                          edgecolor='none')
            self.feature_3d_artists.append(surf)

        # Draw drone as 3D arrow (quiver)
        if self.drone_3d_quiver:
            self.drone_3d_quiver.remove()

        arrow_length = 3.0
        dx = arrow_length * np.cos(self.drone_yaw)
        dy = arrow_length * np.sin(self.drone_yaw)
        dz = 0  # Arrow points horizontally

        self.drone_3d_quiver = self.ax_3d.quiver(
            self.drone_pos[0], self.drone_pos[1], self.drone_pos[2],
            dx, dy, dz,
            color='red', arrow_length_ratio=0.3, linewidth=3, alpha=0.8
        )

        # Draw rover as 3D arrow
        if self.rover_3d_quiver:
            self.rover_3d_quiver.remove()

        arrow_length = 2.0
        dx = arrow_length * np.cos(self.rover_yaw)
        dy = arrow_length * np.sin(self.rover_yaw)
        dz = 0

        self.rover_3d_quiver = self.ax_3d.quiver(
            self.rover_pos[0], self.rover_pos[1], self.rover_pos[2],
            dx, dy, dz,
            color='blue', arrow_length_ratio=0.3, linewidth=2, alpha=0.8
        )

        # Draw 3D trails
        if self.drone_trail_3d:
            self.drone_trail_3d.remove()
        if len(self.drone_trail_x) > 1:
            self.drone_trail_3d = self.ax_3d.plot(
                self.drone_trail_x, self.drone_trail_y, self.drone_trail_z,
                'r-', alpha=0.4, linewidth=2
            )[0]

        if self.rover_trail_3d:
            self.rover_trail_3d.remove()
        if len(self.rover_trail_x) > 1:
            self.rover_trail_3d = self.ax_3d.plot(
                self.rover_trail_x, self.rover_trail_y, self.rover_trail_z,
                'b-', alpha=0.4, linewidth=2
            )[0]


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
