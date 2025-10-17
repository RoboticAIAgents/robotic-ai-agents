"""
Scenario Loader - Load simulation configuration from YAML files.

Responsibilities:
- Parse YAML scenario files
- Validate configuration
- Provide structured scenario data
"""

import yaml
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
from microsim.world import SemanticClass


@dataclass
class RobotInitialPose:
    """Initial pose configuration for a robot."""
    x: float
    y: float
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    theta: float = 0.0  # For 2D robots (rover)


@dataclass
class WorldFeature:
    """A feature/obstacle in the world."""
    type: str  # 'obstacle', 'hazard', 'target', 'water'
    position: List[float]  # [x, y]
    radius: float
    height: float = 0.0


@dataclass
class ScenarioConfig:
    """Complete scenario configuration."""
    # World settings
    world_size: List[float]
    world_resolution: float

    # Robot initial poses
    drone_pose: RobotInitialPose
    rover_pose: RobotInitialPose

    # Robot parameters
    drone_max_velocity: float
    drone_max_accel: float
    rover_max_velocity: float
    rover_max_omega: float

    # Sensor settings
    gps_rate_hz: float
    gps_noise_std: float
    range_rate_hz: float
    range_max_range: float
    range_noise_std: float
    camera_width: int
    camera_height: int
    camera_fov_deg: float
    camera_rate_hz: float

    # Radio settings
    radio_max_range: float
    radio_base_latency: float
    radio_jitter: float
    radio_packet_loss: float

    # World features
    features: List[WorldFeature]


class ScenarioLoader:
    """Load and parse YAML scenario files."""

    @staticmethod
    def load_from_file(filepath: str) -> ScenarioConfig:
        """
        Load scenario from YAML file.

        Args:
            filepath: Path to YAML scenario file

        Returns:
            Parsed ScenarioConfig

        Raises:
            FileNotFoundError: If file doesn't exist
            ValueError: If YAML is invalid or missing required fields
        """
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f"Scenario file not found: {filepath}")
        except yaml.YAMLError as e:
            raise ValueError(f"Invalid YAML in {filepath}: {e}")

        if data is None:
            raise ValueError(f"Empty YAML file: {filepath}")

        return ScenarioLoader._parse_scenario(data)

    @staticmethod
    def _parse_scenario(data: Dict[str, Any]) -> ScenarioConfig:
        """Parse YAML data into ScenarioConfig."""

        # World settings
        world = data.get('world', {})
        world_size = world.get('size', [100.0, 100.0])
        world_resolution = world.get('resolution', 0.5)

        # Robot initial poses
        robots = data.get('robots', {})
        drone_cfg = robots.get('drone', {})
        rover_cfg = robots.get('rover', {})

        drone_init = drone_cfg.get('initial_pose', {})
        drone_pose = RobotInitialPose(
            x=drone_init.get('x', 0.0),
            y=drone_init.get('y', 0.0),
            z=drone_init.get('z', 1.0),
            yaw=drone_init.get('yaw', 0.0),
            roll=drone_init.get('roll', 0.0),
            pitch=drone_init.get('pitch', 0.0)
        )

        rover_init = rover_cfg.get('initial_pose', {})
        rover_pose = RobotInitialPose(
            x=rover_init.get('x', 0.0),
            y=rover_init.get('y', 0.0),
            theta=rover_init.get('theta', 0.0)
        )

        # Robot parameters
        drone_max_velocity = drone_cfg.get('max_velocity', 3.0)
        drone_max_accel = drone_cfg.get('max_accel', 2.0)
        rover_max_velocity = rover_cfg.get('max_velocity', 1.0)
        rover_max_omega = rover_cfg.get('max_omega', 1.57)

        # Sensor settings
        sensors = data.get('sensors', {})
        gps = sensors.get('gps', {})
        range_sensor = sensors.get('range', {})
        camera = sensors.get('camera', {})

        gps_rate_hz = gps.get('rate_hz', 10.0)
        gps_noise_std = gps.get('noise_std', 0.1)

        range_rate_hz = range_sensor.get('rate_hz', 20.0)
        range_max_range = range_sensor.get('max_range', 10.0)
        range_noise_std = range_sensor.get('noise_std', 0.05)

        camera_width = camera.get('width', 128)
        camera_height = camera.get('height', 128)
        camera_fov_deg = camera.get('fov_deg', 90.0)
        camera_rate_hz = camera.get('rate_hz', 2.0)

        # Radio settings
        radio = data.get('radio', {})
        radio_max_range = radio.get('max_range', 100.0)
        radio_base_latency = radio.get('base_latency', 0.05)
        radio_jitter = radio.get('jitter', 0.01)
        radio_packet_loss = radio.get('packet_loss', 0.01)

        # World features
        features_data = data.get('features', [])
        features = []

        for feat in features_data:
            feature = WorldFeature(
                type=feat.get('type', 'obstacle'),
                position=feat.get('position', [0.0, 0.0]),
                radius=feat.get('radius', 1.0),
                height=feat.get('height', 0.0)
            )
            features.append(feature)

        return ScenarioConfig(
            world_size=world_size,
            world_resolution=world_resolution,
            drone_pose=drone_pose,
            rover_pose=rover_pose,
            drone_max_velocity=drone_max_velocity,
            drone_max_accel=drone_max_accel,
            rover_max_velocity=rover_max_velocity,
            rover_max_omega=rover_max_omega,
            gps_rate_hz=gps_rate_hz,
            gps_noise_std=gps_noise_std,
            range_rate_hz=range_rate_hz,
            range_max_range=range_max_range,
            range_noise_std=range_noise_std,
            camera_width=camera_width,
            camera_height=camera_height,
            camera_fov_deg=camera_fov_deg,
            camera_rate_hz=camera_rate_hz,
            radio_max_range=radio_max_range,
            radio_base_latency=radio_base_latency,
            radio_jitter=radio_jitter,
            radio_packet_loss=radio_packet_loss,
            features=features
        )

    @staticmethod
    def feature_type_to_semantic(feature_type: str) -> SemanticClass:
        """Convert feature type string to SemanticClass."""
        mapping = {
            'ground': SemanticClass.GROUND,
            'obstacle': SemanticClass.OBSTACLE,
            'hazard': SemanticClass.HAZARD,
            'target': SemanticClass.TARGET,
            'water': SemanticClass.WATER,
        }
        return mapping.get(feature_type.lower(), SemanticClass.OBSTACLE)
