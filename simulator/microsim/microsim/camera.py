"""
Camera - RGB pinhole camera with semantic rendering.

Responsibilities:
- Ray-cast through pinhole model to render RGB image
- Height-aware occlusion (z-test for nearest surface)
- Fixed 5-color semantic palette
- Publish image_raw (rgb8) and camera_info
"""

import numpy as np
from typing import Tuple


# Fixed RGB color palette for semantic classes (0-255 range)
SEMANTIC_COLORS = {
    0: (150, 120, 90),   # GROUND: brownish
    1: (100, 100, 100),  # OBSTACLE: gray
    2: (255, 255, 0),    # HAZARD: yellow
    3: (0, 255, 0),      # TARGET: green
    4: (0, 150, 255),    # WATER: blue
}


class PinholeCamera:
    """RGB pinhole camera with semantic rendering."""

    def __init__(self, width: int = 128, height: int = 128,
                 fov_deg: float = 90.0, rate_hz: float = 2.0):
        """
        Initialize pinhole camera.

        Args:
            width: Image width in pixels
            height: Image height in pixels
            fov_deg: Horizontal field of view in degrees
            rate_hz: Camera update rate in Hz
        """
        self.width = width
        self.height = height
        self.fov_deg = fov_deg
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.time_since_update = 0.0

        # Compute intrinsic matrix
        self.fx = width / (2.0 * np.tan(np.radians(fov_deg) / 2.0))
        self.fy = self.fx  # Square pixels
        self.cx = width / 2.0
        self.cy = height / 2.0

    def update(self, dt: float) -> bool:
        """
        Check if camera should capture image.

        Args:
            dt: Timestep in seconds

        Returns:
            True if camera should render, False otherwise
        """
        self.time_since_update += dt

        if self.time_since_update >= self.dt:
            self.time_since_update = 0.0
            return True

        return False

    def render(self, world, drone_x: float, drone_y: float, drone_z: float,
               drone_yaw: float, drone_pitch: float) -> np.ndarray:
        """
        Render RGB image from drone camera.

        Uses pinhole camera model with ray-casting to render semantic labels
        as RGB colors. Implements height-aware occlusion (z-test).

        Args:
            world: World object with height_map and semantic_map
            drone_x, drone_y, drone_z: Drone position
            drone_yaw, drone_pitch: Drone orientation

        Returns:
            RGB image as numpy array (height, width, 3) uint8
        """
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Camera is pitched down 15 degrees relative to drone
        camera_pitch = drone_pitch + np.radians(15.0)

        # Precompute rotation matrices
        cos_yaw = np.cos(drone_yaw)
        sin_yaw = np.sin(drone_yaw)
        cos_pitch = np.cos(camera_pitch)
        sin_pitch = np.sin(camera_pitch)

        # For each pixel, cast a ray
        for v in range(self.height):
            for u in range(self.width):
                # Compute ray direction in camera frame (pinhole model)
                # Camera frame: +Z forward, +X right, +Y down
                x_cam = (u - self.cx) / self.fx
                y_cam = (v - self.cy) / self.fy
                z_cam = 1.0

                # Normalize ray direction
                ray_length = np.sqrt(x_cam**2 + y_cam**2 + z_cam**2)
                x_cam /= ray_length
                y_cam /= ray_length
                z_cam /= ray_length

                # Rotate ray to world frame
                # First apply pitch (rotation around x-axis)
                x_pitch = x_cam
                y_pitch = y_cam * cos_pitch - z_cam * sin_pitch
                z_pitch = y_cam * sin_pitch + z_cam * cos_pitch

                # Then apply yaw (rotation around z-axis)
                x_world = x_pitch * cos_yaw - y_pitch * sin_yaw
                y_world = x_pitch * sin_yaw + y_pitch * cos_yaw
                z_world = z_pitch

                # Cast ray and find intersection
                color = self._cast_ray(
                    world,
                    drone_x, drone_y, drone_z,
                    x_world, y_world, z_world
                )

                image[v, u] = color

        return image

    def _cast_ray(self, world, origin_x: float, origin_y: float, origin_z: float,
                  dir_x: float, dir_y: float, dir_z: float) -> Tuple[int, int, int]:
        """
        Cast a ray and return the RGB color of the first intersection.

        Uses height-aware occlusion: checks ray-ground intersection and
        compares with terrain height.

        Args:
            world: World object
            origin_x, origin_y, origin_z: Ray origin (camera position)
            dir_x, dir_y, dir_z: Ray direction (normalized)

        Returns:
            RGB color tuple (r, g, b)
        """
        # Maximum ray length
        max_distance = 100.0
        step_size = world.resolution

        # Cast ray by stepping along direction
        for distance in np.arange(0.0, max_distance, step_size):
            # Current ray position
            ray_x = origin_x + distance * dir_x
            ray_y = origin_y + distance * dir_y
            ray_z = origin_z + distance * dir_z

            # Check if ray has hit the ground/terrain
            terrain_height = world.get_height(ray_x, ray_y)

            if ray_z <= terrain_height:
                # Ray intersected terrain, get semantic class
                semantic = world.get_semantic(ray_x, ray_y)
                return SEMANTIC_COLORS.get(int(semantic), SEMANTIC_COLORS[0])

        # No intersection - return sky color (light blue)
        return (135, 206, 235)  # Sky blue

    def get_camera_info(self) -> dict:
        """
        Get camera intrinsic parameters.

        Returns:
            Dictionary with K, P, distortion_model, D, width, height
        """
        K = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0
        ]

        # Projection matrix P = [K | 0] for monocular camera
        P = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        return {
            'K': K,
            'P': P,
            'distortion_model': 'plumb_bob',
            'D': [0.0, 0.0, 0.0, 0.0, 0.0],
            'width': self.width,
            'height': self.height,
        }
