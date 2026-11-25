"""Start the optical flow node with the perception configuration and the chosen overrides."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Declare the arguments and build the flow node once their values are known."""
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='raft_small',
                              description='Flow backend: farneback, raft_small, raft_large or '
                                          'frame_route.'),
        DeclareLaunchArgument('device', default_value='auto',
                              description="Device for RAFT and frame_route: 'auto', 'cpu' or "
                                          "'cuda'."),
        DeclareLaunchArgument('width', default_value='320',
                              description='Processing width in pixels (height follows aspect).'),
        DeclareLaunchArgument('max_rate', default_value='10.0',
                              description='Largest rate in wall-clock Hz (0: no limit).'),
        DeclareLaunchArgument('num_threads', default_value='0',
                              description='CPU threads for PyTorch (0: the PyTorch default).'),
        DeclareLaunchArgument('publish_vis', default_value='true',
                              description='Publish the flow picture (false saves compute).'),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    """Resolve the arguments to strings and build the flow node."""
    config = os.path.join(get_package_share_directory('rapid_nav_perception'), 'config',
                          'perception.yaml')
    overrides = {
        'use_sim_time': True,
        'backend': LaunchConfiguration('backend').perform(context),
        'device': LaunchConfiguration('device').perform(context),
        'width': int(LaunchConfiguration('width').perform(context)),
        'max_rate': float(LaunchConfiguration('max_rate').perform(context)),
        'num_threads': int(LaunchConfiguration('num_threads').perform(context)),
        'publish_vis':
            LaunchConfiguration('publish_vis').perform(context).strip().lower() == 'true',
    }
    node = Node(
        package='rapid_nav_perception',
        executable='flow_node',
        name='flow_node',
        output='screen',
        parameters=[config, overrides],
    )
    return [node]
