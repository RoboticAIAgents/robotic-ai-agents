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
        Optimized with vectorized NumPy operations.

        Args:
            world: World object with height_map and semantic_map
            drone_x, drone_y, drone_z: Drone position
            drone_yaw, drone_pitch: Drone orientation

        Returns:
            RGB image as numpy array (height, width, 3) uint8
        """
        # Camera is pitched down 15 degrees relative to drone
        camera_pitch = drone_pitch + np.radians(15.0)

        # Precompute rotation matrices
        cos_yaw = np.cos(drone_yaw)
        sin_yaw = np.sin(drone_yaw)
        cos_pitch = np.cos(camera_pitch)
        sin_pitch = np.sin(camera_pitch)

        # Create pixel coordinate grids (vectorized)
        u_coords, v_coords = np.meshgrid(np.arange(self.width), np.arange(self.height))

        # Compute ray directions in camera frame for ALL pixels at once
        x_cam = (u_coords - self.cx) / self.fx
        y_cam = (v_coords - self.cy) / self.fy
        z_cam = np.ones_like(x_cam)

        # Normalize ray directions
        ray_length = np.sqrt(x_cam**2 + y_cam**2 + z_cam**2)
        x_cam /= ray_length
        y_cam /= ray_length
        z_cam /= ray_length

        # Rotate rays to world frame (vectorized)
        # First apply pitch (rotation around x-axis)
        x_pitch = x_cam
        y_pitch = y_cam * cos_pitch - z_cam * sin_pitch
        z_pitch = y_cam * sin_pitch + z_cam * cos_pitch

        # Then apply yaw (rotation around z-axis)
        x_world = x_pitch * cos_yaw - y_pitch * sin_yaw
        y_world = x_pitch * sin_yaw + y_pitch * cos_yaw
        z_world = z_pitch

        # Cast rays for all pixels (vectorized)
        image = self._cast_rays_vectorized(
            world,
            drone_x, drone_y, drone_z,
            x_world, y_world, z_world
        )

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

    def _cast_rays_vectorized(self, world, origin_x: float, origin_y: float, origin_z: float,
                               dir_x: np.ndarray, dir_y: np.ndarray, dir_z: np.ndarray) -> np.ndarray:
        """
        Cast rays for all pixels at once using vectorized NumPy operations.

        This is significantly faster than the per-pixel loop approach.

        Args:
            world: World object
            origin_x, origin_y, origin_z: Ray origin (camera position)
            dir_x, dir_y, dir_z: Ray directions for all pixels (height x width arrays)

        Returns:
            RGB image as numpy array (height, width, 3) uint8
        """
        height, width = dir_x.shape
        image = np.zeros((height, width, 3), dtype=np.uint8)

        # Sky color for all pixels initially
        sky_color = np.array([135, 206, 235], dtype=np.uint8)
        image[:, :] = sky_color

        # Maximum ray length and step size
        max_distance = 100.0
        step_size = world.resolution
        num_steps = int(max_distance / step_size)

        # Mask to track which pixels still need to be rendered
        active_mask = np.ones((height, width), dtype=bool)

        # Step along all rays simultaneously
        for step in range(num_steps):
            if not active_mask.any():
                break  # All rays have hit something

            distance = step * step_size

            # Compute ray positions for active pixels only
            ray_x = origin_x + distance * dir_x[active_mask]
            ray_y = origin_y + distance * dir_y[active_mask]
            ray_z = origin_z + distance * dir_z[active_mask]

            # Get terrain heights (vectorized)
            # Convert to grid indices using World's coordinate system
            # World grid is centered: origin is at (-size[0]/2, -size[1]/2)
            grid_i = ((ray_x + world.size[0]/2) / world.resolution).astype(int)
            grid_j = ((ray_y + world.size[1]/2) / world.resolution).astype(int)

            # Clamp to valid range
            grid_i = np.clip(grid_i, 0, world.grid_shape[0] - 1)
            grid_j = np.clip(grid_j, 0, world.grid_shape[1] - 1)

            # Get terrain heights and semantic labels
            terrain_heights = world.height_map[grid_j, grid_i]

            # Check which rays hit terrain
            hit_mask = ray_z <= terrain_heights

            if hit_mask.any():
                # Get semantic labels for hits
                semantic_labels = world.semantic_map[grid_j[hit_mask], grid_i[hit_mask]]

                # Convert semantic labels to RGB colors
                for i, semantic in enumerate(semantic_labels):
                    color = SEMANTIC_COLORS.get(int(semantic), SEMANTIC_COLORS[0])
                    # Find the position in the full image where this hit occurred
                    hit_indices = np.where(active_mask)
                    local_hit_idx = np.where(hit_mask)[0][i]
                    row_idx = hit_indices[0][local_hit_idx]
                    col_idx = hit_indices[1][local_hit_idx]
                    image[row_idx, col_idx] = color

                # Mark these pixels as no longer active
                active_indices = np.argwhere(active_mask)
                hit_positions = active_indices[hit_mask]
                for pos in hit_positions:
                    active_mask[pos[0], pos[1]] = False

        return image

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
