"""
Launch file for MicroSim integration tests.

This launch file starts the MicroSim node and runs integration tests against it.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import launch_testing
import launch_testing.actions


def generate_launch_description():
    """Generate launch description for integration tests."""

    # Get the package directory
    pkg_dir = Path(__file__).parent.parent
    scenario_file = pkg_dir / 'scenarios' / 'default.yaml'

    # Declare launch arguments
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value=str(scenario_file),
        description='Path to scenario YAML file'
    )

    # MicroSim node
    microsim_node = Node(
        package='microsim',
        executable='microsim_node',
        name='microsim',
        output='screen',
        parameters=[{
            'scenario_file': LaunchConfiguration('scenario')
        }]
    )

    # Integration test process
    test_process = launch_testing.actions.ReadyToTest()

    return LaunchDescription([
        scenario_arg,
        microsim_node,
        test_process
    ])
