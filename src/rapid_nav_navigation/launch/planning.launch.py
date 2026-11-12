"""Start the planning stack: the map, the planner servers with our planner, and the localisation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BUILT_IN_MAPS = ('arena', 'maze')


def generate_launch_description():
    """Declare the arguments; the nodes are built once their values are known."""
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='arena',
                              description="'arena', 'maze' or the path of a map YAML."),
        DeclareLaunchArgument('localization', default_value='amcl',
                              description="'amcl' (lidar localisation) or 'static' (no simulation)."),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the simulated clock.'),
        DeclareLaunchArgument('x', default_value='0.0', description='Initial pose x (m).'),
        DeclareLaunchArgument('y', default_value='0.0', description='Initial pose y (m).'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial pose yaw (rad).'),
        OpaqueFunction(function=launch_setup),
    ])


def map_yaml_path(name):
    """Return the YAML of a built-in map, or the given path."""
    if name in BUILT_IN_MAPS:
        return os.path.join(get_package_share_directory('rapid_nav_navigation'), 'maps', name + '.yaml')
    return name


def launch_setup(context):
    """Resolve the arguments and build the nodes of the planning stack."""
    share = get_package_share_directory('rapid_nav_navigation')
    config = os.path.join(share, 'config', 'planning.yaml')
    map_yaml = map_yaml_path(LaunchConfiguration('map').perform(context))
    localization = LaunchConfiguration('localization').perform(context).strip().lower()
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    if localization not in ('amcl', 'static'):
        raise RuntimeError("localization must be 'amcl' or 'static', got '{}'".format(localization))

    nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[config, {'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
    ]
    managed = ['map_server', 'planner_server']
    if localization == 'static':
        # Without a simulation, the robot's pose is fixed: map -> odom is the identity and odom ->
        # base_footprint is the initial pose.
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        ))
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['--x', x, '--y', y, '--yaw', yaw, '--frame-id', 'odom',
                       '--child-frame-id', 'base_footprint'],
        ))
    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': managed}],
    ))
    return nodes
