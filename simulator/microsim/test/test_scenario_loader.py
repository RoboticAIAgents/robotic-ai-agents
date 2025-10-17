"""Unit tests for Scenario Loader module."""

import pytest
import tempfile
import os
from microsim.scenario_loader import ScenarioLoader, ScenarioConfig
from microsim.world import SemanticClass


@pytest.mark.unit
class TestScenarioLoader:
    """Test YAML scenario loading."""

    def test_load_default_scenario(self):
        """Test loading the default scenario file."""
        # Assuming default.yaml exists in scenarios/
        scenario_path = os.path.join(
            os.path.dirname(__file__), '..', 'scenarios', 'default.yaml'
        )

        if not os.path.exists(scenario_path):
            pytest.skip("Default scenario file not found")

        config = ScenarioLoader.load_from_file(scenario_path)

        assert isinstance(config, ScenarioConfig)
        assert config.world_size == [100.0, 100.0]
        assert config.world_resolution == 0.5

    def test_parse_world_settings(self):
        """Test parsing world configuration."""
        yaml_content = """
world:
  size: [50.0, 50.0]
  resolution: 1.0
"""
        config = self._load_from_string(yaml_content)

        assert config.world_size == [50.0, 50.0]
        assert config.world_resolution == 1.0

    def test_parse_robot_initial_poses(self):
        """Test parsing robot initial poses."""
        yaml_content = """
robots:
  drone:
    initial_pose:
      x: 5.0
      y: 10.0
      z: 2.0
      yaw: 1.57
  rover:
    initial_pose:
      x: -3.0
      y: 4.0
      theta: 0.785
"""
        config = self._load_from_string(yaml_content)

        assert config.drone_pose.x == 5.0
        assert config.drone_pose.y == 10.0
        assert config.drone_pose.z == 2.0
        assert config.drone_pose.yaw == 1.57

        assert config.rover_pose.x == -3.0
        assert config.rover_pose.y == 4.0
        assert config.rover_pose.theta == 0.785

    def test_parse_robot_parameters(self):
        """Test parsing robot dynamics parameters."""
        yaml_content = """
robots:
  drone:
    max_velocity: 5.0
    max_accel: 3.0
  rover:
    max_velocity: 2.0
    max_omega: 2.0
"""
        config = self._load_from_string(yaml_content)

        assert config.drone_max_velocity == 5.0
        assert config.drone_max_accel == 3.0
        assert config.rover_max_velocity == 2.0
        assert config.rover_max_omega == 2.0

    def test_parse_sensor_settings(self):
        """Test parsing sensor configurations."""
        yaml_content = """
sensors:
  gps:
    rate_hz: 5.0
    noise_std: 0.2
  range:
    rate_hz: 15.0
    max_range: 8.0
    noise_std: 0.1
  camera:
    width: 256
    height: 256
    fov_deg: 120.0
    rate_hz: 5.0
"""
        config = self._load_from_string(yaml_content)

        assert config.gps_rate_hz == 5.0
        assert config.gps_noise_std == 0.2

        assert config.range_rate_hz == 15.0
        assert config.range_max_range == 8.0
        assert config.range_noise_std == 0.1

        assert config.camera_width == 256
        assert config.camera_height == 256
        assert config.camera_fov_deg == 120.0
        assert config.camera_rate_hz == 5.0

    def test_parse_radio_settings(self):
        """Test parsing radio link configuration."""
        yaml_content = """
radio:
  max_range: 150.0
  base_latency: 0.1
  jitter: 0.02
  packet_loss: 0.05
"""
        config = self._load_from_string(yaml_content)

        assert config.radio_max_range == 150.0
        assert config.radio_base_latency == 0.1
        assert config.radio_jitter == 0.02
        assert config.radio_packet_loss == 0.05

    def test_parse_world_features(self):
        """Test parsing world features/obstacles."""
        yaml_content = """
features:
  - type: obstacle
    position: [10.0, 10.0]
    radius: 3.0
    height: 5.0
  - type: hazard
    position: [-5.0, 15.0]
    radius: 4.0
    height: 0.0
  - type: target
    position: [20.0, -10.0]
    radius: 2.0
"""
        config = self._load_from_string(yaml_content)

        assert len(config.features) == 3

        assert config.features[0].type == 'obstacle'
        assert config.features[0].position == [10.0, 10.0]
        assert config.features[0].radius == 3.0
        assert config.features[0].height == 5.0

        assert config.features[1].type == 'hazard'
        assert config.features[2].type == 'target'

    def test_default_values(self):
        """Test that missing fields use sensible defaults."""
        yaml_content = """
world:
  size: [100.0, 100.0]
"""
        config = self._load_from_string(yaml_content)

        # Should have defaults for everything
        assert config.world_resolution == 0.5
        assert config.drone_pose.x == 0.0
        assert config.drone_pose.z == 1.0
        assert config.gps_rate_hz == 10.0

    def test_feature_type_to_semantic(self):
        """Test conversion from feature type to semantic class."""
        assert ScenarioLoader.feature_type_to_semantic('obstacle') == SemanticClass.OBSTACLE
        assert ScenarioLoader.feature_type_to_semantic('hazard') == SemanticClass.HAZARD
        assert ScenarioLoader.feature_type_to_semantic('target') == SemanticClass.TARGET
        assert ScenarioLoader.feature_type_to_semantic('water') == SemanticClass.WATER
        assert ScenarioLoader.feature_type_to_semantic('ground') == SemanticClass.GROUND

        # Unknown types default to obstacle
        assert ScenarioLoader.feature_type_to_semantic('unknown') == SemanticClass.OBSTACLE

    def test_missing_file_error(self):
        """Test that loading non-existent file raises error."""
        with pytest.raises(FileNotFoundError):
            ScenarioLoader.load_from_file('/nonexistent/path/scenario.yaml')

    def test_empty_yaml(self):
        """Test that empty YAML file raises error."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write('')
            temp_path = f.name

        try:
            with pytest.raises(ValueError, match="Empty YAML"):
                ScenarioLoader.load_from_file(temp_path)
        finally:
            os.unlink(temp_path)

    def _load_from_string(self, yaml_content: str) -> ScenarioConfig:
        """Helper to load scenario from YAML string."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(yaml_content)
            temp_path = f.name

        try:
            config = ScenarioLoader.load_from_file(temp_path)
        finally:
            os.unlink(temp_path)

        return config
