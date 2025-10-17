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

        Args:
            world: World object with height_map and semantic_map
            drone_x, drone_y, drone_z: Drone position
            drone_yaw, drone_pitch: Drone orientation

        Returns:
            RGB image as numpy array (height, width, 3) uint8
        """
        # Placeholder: return blank image for now
        # TODO: Implement ray-casting in Sprint 3
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Fill with ground color as placeholder
        image[:, :] = SEMANTIC_COLORS[0]

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
