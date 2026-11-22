"""Start the moving obstacles of a scenario: the Gazebo bridge for them and the obstacle driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    """Declare the scenario argument; the nodes are built once it is known."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario',
            default_value='',
            description='Scenario of nav_scenarios.yaml whose moving obstacles to start '
                        '(none when empty or when the scenario has no obstacles).',
        ),
        DeclareLaunchArgument(
            'hazards',
            default_value='off',
            description="'oracle' or 'oracle_static' also starts oracle_obstacles for the "
                        "scenario (true or zero velocities); 'off' and 'flow' start nothing.",
        ),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    """Return the bridge and the driver for the scenario's obstacles, or nothing."""
    name = LaunchConfiguration('scenario').perform(context).strip()
    if not name:
        return []
    path = os.path.join(get_package_share_directory('rapid_nav_navigation'), 'config',
                        'nav_scenarios.yaml')
    with open(path, encoding='utf-8') as handle:
        scenarios = yaml.safe_load(handle)
    if name not in scenarios:
        raise RuntimeError("unknown scenario '{}'; known: {}".format(
            name, ', '.join(sorted(scenarios))))
    obstacles = scenarios[name].get('obstacles') or []
    if not obstacles:
        return []
    topics = []
    for obstacle in obstacles:
        topics.append('/model/{}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'.format(
            obstacle['name']))
        topics.append('/model/{}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'.format(
            obstacle['name']))
    hazards = LaunchConfiguration('hazards').perform(context).strip().lower()
    if hazards not in ('off', 'oracle', 'oracle_static', 'flow'):
        raise RuntimeError(
            "hazards must be 'off', 'oracle', 'oracle_static' or 'flow', got '{}'".format(hazards))
    oracle = []
    if hazards in ('oracle', 'oracle_static'):
        oracle = [Node(
            package='rapid_nav_tools',
            executable='oracle_obstacles',
            name='oracle_obstacles',
            output='screen',
            parameters=[{'scenario': name,
                         'velocity_mode': 'true' if hazards == 'oracle' else 'zero',
                         'use_sim_time': True}],
        )]
    return oracle + [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='obstacle_bridge',
            output='screen',
            arguments=topics,
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rapid_nav_tools',
            executable='obstacle_driver',
            name='obstacle_driver',
            output='screen',
            parameters=[{'scenario': name, 'use_sim_time': True}],
        ),
    ]
