"""Unit tests for Camera module."""

import pytest
import numpy as np
from microsim.camera import PinholeCamera, SEMANTIC_COLORS
from microsim.world import World, SemanticClass


@pytest.mark.unit
class TestPinholeCamera:
    """Test RGB pinhole camera rendering."""

    def test_initialization(self):
        """Test camera initializes with correct parameters."""
        camera = PinholeCamera(width=128, height=128, fov_deg=90.0, rate_hz=2.0)

        assert camera.width == 128
        assert camera.height == 128
        assert camera.fov_deg == 90.0
        assert camera.rate_hz == 2.0
        assert camera.dt == 0.5

    def test_intrinsic_matrix(self):
        """Test that intrinsic matrix is computed correctly."""
        camera = PinholeCamera(width=128, height=128, fov_deg=90.0)

        # For 90 degree FOV: fx = width / (2 * tan(45°)) = 128 / 2 = 64
        expected_fx = 128.0 / (2.0 * np.tan(np.radians(45.0)))

        assert abs(camera.fx - expected_fx) < 0.01
        assert camera.fy == camera.fx  # Square pixels
        assert camera.cx == 64.0  # width / 2
        assert camera.cy == 64.0  # height / 2

    def test_update_timing(self):
        """Test camera update timing."""
        camera = PinholeCamera(rate_hz=2.0)  # 2 Hz = 0.5s period

        # First update - not ready
        assert camera.update(dt=0.3) == False

        # Second update - ready (total 0.5s)
        assert camera.update(dt=0.2) == True

        # Third update - not ready again
        assert camera.update(dt=0.1) == False

    def test_render_flat_ground(self):
        """Test rendering a flat ground scene."""
        camera = PinholeCamera(width=64, height=64)
        world = World(size=(100.0, 100.0), resolution=1.0)

        # Render from position looking down
        image = camera.render(
            world,
            drone_x=0.0,
            drone_y=0.0,
            drone_z=5.0,
            drone_yaw=0.0,
            drone_pitch=0.0
        )

        assert image.shape == (64, 64, 3)
        assert image.dtype == np.uint8

        # Most pixels should be ground color (brownish) since looking down
        ground_color = SEMANTIC_COLORS[SemanticClass.GROUND]
        # Check center pixel
        assert tuple(image[32, 32]) == ground_color or tuple(image[32, 32]) == (135, 206, 235)  # ground or sky

    def test_render_with_obstacle(self):
        """Test that camera can render obstacles."""
        camera = PinholeCamera(width=32, height=32)
        world = World(size=(100.0, 100.0), resolution=0.5)

        # Place a very large obstacle all around
        world.set_region(0.0, 0.0, radius=20.0, height=10.0, semantic=SemanticClass.OBSTACLE)

        # Render from inside the obstacle looking down
        image = camera.render(
            world,
            drone_x=0.0,
            drone_y=0.0,
            drone_z=5.0,
            drone_yaw=0.0,
            drone_pitch=np.radians(80.0)  # Almost straight down
        )

        # Most of image should show obstacle (not sky)
        obstacle_pixels = 0
        sky_pixels = 0
        for y in range(32):
            for x in range(32):
                color = tuple(image[y, x])
                if color == (100, 100, 100):  # Obstacle color
                    obstacle_pixels += 1
                elif color == (135, 206, 235):  # Sky color
                    sky_pixels += 1

        # Should have rendered something (not all sky)
        assert obstacle_pixels > 0 or sky_pixels < 32*32

    def test_semantic_color_mapping(self):
        """Test that different semantic classes produce different colors."""
        camera = PinholeCamera(width=16, height=16)
        world = World(size=(50.0, 50.0), resolution=0.5)

        # Place different semantic regions at ground level
        world.set_region(10.0, 0.0, radius=2.0, height=2.0, semantic=SemanticClass.OBSTACLE)
        world.set_region(0.0, 10.0, radius=2.0, height=0.1, semantic=SemanticClass.HAZARD)
        world.set_region(-10.0, 0.0, radius=2.0, height=0.1, semantic=SemanticClass.TARGET)

        # Render looking down to see the ground
        image = camera.render(world, 0.0, 0.0, 3.0, 0.0, np.radians(60.0))  # Pitched down significantly

        # Image should contain multiple colors
        unique_colors = set()
        for y in range(16):
            for x in range(16):
                unique_colors.add(tuple(image[y, x]))

        # Should have at least 2 different colors (ground + something else)
        assert len(unique_colors) >= 2

    def test_sky_rendering(self):
        """Test that sky is rendered when ray doesn't hit ground."""
        camera = PinholeCamera(width=32, height=32)
        world = World(size=(100.0, 100.0), resolution=1.0)

        # Render from high altitude looking up (negative pitch)
        image = camera.render(
            world,
            drone_x=0.0,
            drone_y=0.0,
            drone_z=50.0,
            drone_yaw=0.0,
            drone_pitch=-np.radians(30.0)  # Looking up
        )

        # Top pixels should be sky blue
        sky_color = (135, 206, 235)
        top_pixel = tuple(image[0, 16])

        # Top should be sky or similar
        assert top_pixel == sky_color or top_pixel[2] > 200  # Blueish

    def test_get_camera_info(self):
        """Test camera info dictionary."""
        camera = PinholeCamera(width=128, height=128, fov_deg=90.0)

        info = camera.get_camera_info()

        assert 'K' in info
        assert 'P' in info
        assert 'D' in info
        assert 'distortion_model' in info
        assert 'width' in info
        assert 'height' in info

        assert len(info['K']) == 9
        assert len(info['P']) == 12
        assert len(info['D']) == 5

        assert info['width'] == 128
        assert info['height'] == 128
        assert info['distortion_model'] == 'plumb_bob'

    def test_occlusion(self):
        """Test height-aware occlusion."""
        camera = PinholeCamera(width=16, height=16)
        world = World(size=(50.0, 50.0), resolution=0.5)

        # Place a tall obstacle close, and a shorter one behind it
        world.set_region(5.0, 0.0, radius=1.0, height=10.0, semantic=SemanticClass.OBSTACLE)
        world.set_region(15.0, 0.0, radius=3.0, height=2.0, semantic=SemanticClass.TARGET)

        # Render looking forward
        image = camera.render(world, 0.0, 0.0, 3.0, 0.0, 0.0)

        # The tall close obstacle should occlude the far target
        # Center should show obstacle, not target
        center_color = tuple(image[8, 8])
        obstacle_color = SEMANTIC_COLORS[SemanticClass.OBSTACLE]
        target_color = SEMANTIC_COLORS[SemanticClass.TARGET]

        # Should see obstacle or ground, not target (due to occlusion)
        assert center_color != target_color

    def test_camera_pitch_affects_view(self):
        """Test that camera pitch changes the view."""
        camera = PinholeCamera(width=32, height=32)
        world = World(size=(100.0, 100.0), resolution=1.0)

        # Place obstacle at ground level
        world.set_region(10.0, 0.0, radius=3.0, height=2.0, semantic=SemanticClass.OBSTACLE)

        # Render with different pitches
        image_level = camera.render(world, 0.0, 0.0, 5.0, 0.0, 0.0)
        image_down = camera.render(world, 0.0, 0.0, 5.0, 0.0, np.radians(45.0))

        # Images should be different
        assert not np.array_equal(image_level, image_down)


@pytest.mark.unit
class TestSemanticColors:
    """Test semantic color palette."""

    def test_all_classes_have_colors(self):
        """Test that all semantic classes have defined colors."""
        for semantic_class in [0, 1, 2, 3, 4]:
            assert semantic_class in SEMANTIC_COLORS
            color = SEMANTIC_COLORS[semantic_class]
            assert len(color) == 3
            assert all(0 <= c <= 255 for c in color)

    def test_colors_are_distinct(self):
        """Test that semantic colors are visually distinct."""
        colors = list(SEMANTIC_COLORS.values())

        # All colors should be different
        assert len(colors) == len(set(colors))
