"""Unit tests for World module."""

import pytest
import numpy as np
from microsim.world import World, SemanticClass


@pytest.mark.unit
class TestWorld:
    """Test 2D grid world representation."""

    def test_initialization(self):
        """Test world initializes with correct grid size."""
        world = World(size=(100.0, 100.0), resolution=0.5)

        assert world.size == (100.0, 100.0)
        assert world.resolution == 0.5
        assert world.grid_shape == (200, 200)  # 100/0.5 = 200

    def test_default_height_is_zero(self):
        """Test that default height map is all zeros."""
        world = World(size=(10.0, 10.0), resolution=1.0)

        # Check center and corners
        assert world.get_height(0.0, 0.0) == 0.0
        assert world.get_height(4.0, 4.0) == 0.0
        assert world.get_height(-4.0, -4.0) == 0.0

    def test_default_semantic_is_ground(self):
        """Test that default semantic map is all GROUND."""
        world = World(size=(10.0, 10.0), resolution=1.0)

        assert world.get_semantic(0.0, 0.0) == SemanticClass.GROUND
        assert world.get_semantic(3.0, 3.0) == SemanticClass.GROUND

    def test_out_of_bounds_returns_defaults(self):
        """Test that out-of-bounds queries return safe defaults."""
        world = World(size=(10.0, 10.0), resolution=1.0)

        # Query outside world bounds
        assert world.get_height(100.0, 100.0) == 0.0
        assert world.get_semantic(100.0, 100.0) == SemanticClass.GROUND

    def test_set_region_obstacle(self):
        """Test setting a circular obstacle region."""
        world = World(size=(100.0, 100.0), resolution=0.5)

        # Place obstacle at (10, 10) with radius 2m, height 5m
        world.set_region(10.0, 10.0, radius=2.0, height=5.0, semantic=SemanticClass.OBSTACLE)

        # Check center is obstacle
        assert world.get_semantic(10.0, 10.0) == SemanticClass.OBSTACLE
        assert world.get_height(10.0, 10.0) == 5.0

        # Check point inside radius
        assert world.get_semantic(11.0, 10.0) == SemanticClass.OBSTACLE

        # Check point outside radius
        assert world.get_semantic(15.0, 10.0) == SemanticClass.GROUND
        assert world.get_height(15.0, 10.0) == 0.0

    def test_set_region_hazard(self):
        """Test setting a hazard zone."""
        world = World(size=(50.0, 50.0), resolution=1.0)

        world.set_region(-5.0, -5.0, radius=3.0, height=0.0, semantic=SemanticClass.HAZARD)

        assert world.get_semantic(-5.0, -5.0) == SemanticClass.HAZARD
        assert world.get_semantic(-6.0, -5.0) == SemanticClass.HAZARD
        assert world.get_semantic(-10.0, -5.0) == SemanticClass.GROUND

    def test_set_region_target(self):
        """Test setting a target zone."""
        world = World(size=(50.0, 50.0), resolution=1.0)

        world.set_region(20.0, -10.0, radius=5.0, height=0.0, semantic=SemanticClass.TARGET)

        assert world.get_semantic(20.0, -10.0) == SemanticClass.TARGET
        assert world.get_semantic(22.0, -10.0) == SemanticClass.TARGET

    def test_set_region_water(self):
        """Test setting a water body."""
        world = World(size=(50.0, 50.0), resolution=0.5)

        world.set_region(0.0, 0.0, radius=10.0, height=0.0, semantic=SemanticClass.WATER)

        assert world.get_semantic(0.0, 0.0) == SemanticClass.WATER
        assert world.get_semantic(5.0, 5.0) == SemanticClass.WATER

    def test_multiple_regions(self):
        """Test placing multiple regions."""
        world = World(size=(100.0, 100.0), resolution=1.0)

        # Place obstacle
        world.set_region(10.0, 10.0, radius=3.0, height=5.0, semantic=SemanticClass.OBSTACLE)

        # Place hazard
        world.set_region(-10.0, -10.0, radius=2.0, height=0.0, semantic=SemanticClass.HAZARD)

        # Check both exist
        assert world.get_semantic(10.0, 10.0) == SemanticClass.OBSTACLE
        assert world.get_height(10.0, 10.0) == 5.0

        assert world.get_semantic(-10.0, -10.0) == SemanticClass.HAZARD
        assert world.get_height(-10.0, -10.0) == 0.0

        # Check they don't interfere
        assert world.get_semantic(0.0, 0.0) == SemanticClass.GROUND

    def test_overlapping_regions(self):
        """Test that later regions overwrite earlier ones."""
        world = World(size=(50.0, 50.0), resolution=0.5)

        # Place ground-level target
        world.set_region(0.0, 0.0, radius=5.0, height=0.0, semantic=SemanticClass.TARGET)

        # Overwrite center with obstacle
        world.set_region(0.0, 0.0, radius=2.0, height=3.0, semantic=SemanticClass.OBSTACLE)

        # Center should be obstacle
        assert world.get_semantic(0.0, 0.0) == SemanticClass.OBSTACLE
        assert world.get_height(0.0, 0.0) == 3.0

        # Edge should still be target
        assert world.get_semantic(4.0, 0.0) == SemanticClass.TARGET

    def test_world_to_grid_conversion(self):
        """Test coordinate conversion from world to grid."""
        world = World(size=(100.0, 100.0), resolution=1.0)

        # Center of world (0, 0) should map to center of grid
        i, j = world._world_to_grid(0.0, 0.0)
        assert i == 50
        assert j == 50

        # Corner
        i, j = world._world_to_grid(-50.0, -50.0)
        assert i == 0
        assert j == 0

    def test_in_bounds_check(self):
        """Test grid bounds checking."""
        world = World(size=(10.0, 10.0), resolution=1.0)

        assert world._in_bounds(0, 0) == True
        assert world._in_bounds(5, 5) == True
        assert world._in_bounds(9, 9) == True
        assert world._in_bounds(-1, 0) == False
        assert world._in_bounds(0, -1) == False
        assert world._in_bounds(10, 5) == False
        assert world._in_bounds(5, 10) == False

    def test_fine_resolution(self):
        """Test world with fine resolution."""
        world = World(size=(10.0, 10.0), resolution=0.1)

        assert world.grid_shape == (100, 100)

        # Place small obstacle
        world.set_region(0.0, 0.0, radius=0.5, height=2.0, semantic=SemanticClass.OBSTACLE)

        assert world.get_semantic(0.0, 0.0) == SemanticClass.OBSTACLE
        assert world.get_semantic(0.3, 0.0) == SemanticClass.OBSTACLE
        assert world.get_semantic(1.0, 0.0) == SemanticClass.GROUND

    def test_coarse_resolution(self):
        """Test world with coarse resolution."""
        world = World(size=(100.0, 100.0), resolution=5.0)

        assert world.grid_shape == (20, 20)

        # Place large obstacle
        world.set_region(0.0, 0.0, radius=10.0, height=8.0, semantic=SemanticClass.OBSTACLE)

        assert world.get_semantic(0.0, 0.0) == SemanticClass.OBSTACLE


@pytest.mark.unit
class TestSemanticClass:
    """Test semantic class enum."""

    def test_semantic_values(self):
        """Test that semantic classes have expected integer values."""
        assert SemanticClass.GROUND == 0
        assert SemanticClass.OBSTACLE == 1
        assert SemanticClass.HAZARD == 2
        assert SemanticClass.TARGET == 3
        assert SemanticClass.WATER == 4

    def test_semantic_from_int(self):
        """Test converting int to SemanticClass."""
        assert SemanticClass(0) == SemanticClass.GROUND
        assert SemanticClass(1) == SemanticClass.OBSTACLE
        assert SemanticClass(4) == SemanticClass.WATER
