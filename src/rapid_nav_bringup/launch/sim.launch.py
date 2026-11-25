"""Start the whole simulation: Gazebo with the arena and the robot, the odometry and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ODOM_TF_MODES = ('ekf', 'wheel')
CONTROLLERS = ('dwa', 'dwb', 'rpp')


def generate_launch_description():
    """Declare the arguments; the nodes are built once their values are known."""
    gazebo_share = get_package_share_directory('rapid_nav_gazebo')
    arguments = [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(gazebo_share, 'worlds', 'arena.sdf'),
            description='SDF world to load (default: the arena).',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run without a window and without RViz (software rendering).',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz with the sim.rviz configuration (not when headless).',
        ),
        DeclareLaunchArgument(
            'odom_tf',
            default_value='ekf',
            description="Source of odom -> base_footprint: 'ekf' (robot_localization) or "
                        "'wheel' (the DiffDrive transform from Gazebo).",
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x position (m).'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y position (m).'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn heading (rad).'),
        DeclareLaunchArgument(
            'planning',
            default_value='false',
            description='Start the planning stack: AMCL on the arena map, the planner servers.',
        ),
        DeclareLaunchArgument(
            'navigation',
            default_value='false',
            description='Start the navigation stack (planning, controller, behaviour and BT '
                        'navigator); the goals go to "2D Goal Pose" in RViz.',
        ),
        DeclareLaunchArgument(
            'controller',
            default_value='dwa',
            description="Controller of the navigation stack: 'dwa' (ours), 'dwb' or 'rpp'.",
        ),
        DeclareLaunchArgument(
            'scenario',
            default_value='',
            description='Scenario of nav_scenarios.yaml whose moving obstacles to start '
                        '(empty: none).',
        ),
        DeclareLaunchArgument(
            'rtf',
            default_value='1.0',
            description='Target real-time factor of the simulation (1.0: the world as it is; '
                        'below 1.0 it is set through the world service, so that slow '
                        'perception is fresh in simulated time).',
        ),
        DeclareLaunchArgument(
            'hazards',
            default_value='off',
            description="Hazard layer in the costmaps: 'off', 'oracle' (true obstacle "
                        "velocities), 'oracle_static' (positions only; the ablation), 'flow' "
                        "(camera tracks) or 'fused' (lidar and camera tracks in one filter, with "
                        'uncertainty, and the speed governor).',
        ),
        DeclareLaunchArgument(
            'governor',
            default_value='true',
            description="With hazards 'fused': start the time-to-collision speed governor "
                        "('false': the ablation without it).",
        ),
        DeclareLaunchArgument(
            'perception',
            default_value='false',
            description='Start the optical flow node (rapid_nav_perception) on the camera.',
        ),
        DeclareLaunchArgument(
            'flow_backend',
            default_value='raft_small',
            description='Flow backend: farneback, raft_small, raft_large or frame_route.',
        ),
        DeclareLaunchArgument(
            'flow_device',
            default_value='auto',
            description="Device for RAFT: 'auto', 'cpu' or 'cuda'.",
        ),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    """Build the nodes for the resolved odom_tf, rviz and headless arguments."""
    share = get_package_share_directory('rapid_nav_bringup')
    gazebo_share = get_package_share_directory('rapid_nav_gazebo')

    odom_tf = LaunchConfiguration('odom_tf').perform(context).strip().lower()
    if odom_tf not in ODOM_TF_MODES:
        raise RuntimeError(
            "odom_tf must be 'ekf' or 'wheel', got '{}'".format(odom_tf)
        )
    headless = LaunchConfiguration('headless').perform(context).strip().lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).strip().lower() == 'true'
    world = LaunchConfiguration('world').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments=[
            ('world', world),
            ('headless', 'true' if headless else 'false'),
            ('x', x),
            ('y', y),
            ('yaw', yaw),
        ],
    )
    nodes = [gazebo]
    rtf = float(LaunchConfiguration('rtf').perform(context))
    if rtf <= 0.0:
        raise RuntimeError('rtf must be positive, got {}'.format(rtf))
    if rtf != 1.0:
        # Wait for the world service of the running simulation, then set the target.
        nodes.append(ExecuteProcess(
            cmd=['bash', '-c',
                 'for i in $(seq 1 90); do gz service -s /world/arena/set_physics '
                 '--reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 2000 '
                 '--req "real_time_factor: {}" 2>/dev/null | grep -q true && exit 0; sleep 2; '
                 'done; echo "rtf: set_physics did not answer" >&2; exit 1'.format(rtf)],
            output='screen'))
    planning = LaunchConfiguration('planning').perform(context).strip().lower() == 'true'
    navigation = LaunchConfiguration('navigation').perform(context).strip().lower() == 'true'
    controller = LaunchConfiguration('controller').perform(context).strip().lower()
    if planning and navigation:
        raise RuntimeError(
            'navigation:=true already starts the planning stack; do not pass planning:=true too'
        )
    hazards = LaunchConfiguration('hazards').perform(context).strip().lower()
    if hazards not in ('off', 'oracle', 'oracle_static', 'flow', 'fused'):
        raise RuntimeError(
            "hazards must be 'off', 'oracle', 'oracle_static', 'flow' or 'fused', got '{}'".format(
                hazards)
        )
    governor = LaunchConfiguration('governor').perform(context).strip().lower()
    if governor not in ('true', 'false'):
        raise RuntimeError("governor must be 'true' or 'false', got '{}'".format(governor))
    if controller not in CONTROLLERS:
        raise RuntimeError(
            "controller must be 'dwa', 'dwb' or 'rpp', got '{}'".format(controller)
        )
    if navigation:
        navigation_launch = os.path.join(
            get_package_share_directory('rapid_nav_navigation'), 'launch', 'navigation.launch.py')
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments=[
                ('controller', controller),
                ('hazards', hazards),
                ('governor', governor),
                ('use_sim_time', 'true'),
                ('x', x),
                ('y', y),
                ('yaw', yaw),
            ],
        ))
    if planning:
        planning_launch = os.path.join(
            get_package_share_directory('rapid_nav_navigation'), 'launch', 'planning.launch.py')
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planning_launch),
            launch_arguments=[
                ('localization', 'amcl'),
                ('use_sim_time', 'true'),
                ('x', x),
                ('y', y),
                ('yaw', yaw),
            ],
        ))

    scenario = LaunchConfiguration('scenario').perform(context).strip()
    if scenario:
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'scenario.launch.py')),
            launch_arguments=[('scenario', scenario), ('hazards', hazards)],
        ))

    if odom_tf == 'ekf':
        nodes.append(Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(share, 'config', 'ekf.yaml')],
        ))
    else:
        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_wheel_tf_bridge',
            output='screen',
            parameters=[{
                'config_file': os.path.join(gazebo_share, 'config', 'bridge_wheel_tf.yaml'),
                'use_sim_time': True,
            }],
        ))

    perception = LaunchConfiguration('perception').perform(context).strip().lower() == 'true'
    flow_hazards = hazards in ('flow', 'fused')
    fused = hazards == 'fused'
    if perception or flow_hazards:
        perception_launch = os.path.join(
            get_package_share_directory('rapid_nav_perception'), 'launch', 'perception.launch.py')
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments=[
                ('backend', LaunchConfiguration('flow_backend').perform(context)),
                ('device', LaunchConfiguration('flow_device').perform(context)),
            ] + ([('num_threads', '4'), ('publish_vis', 'false')] if flow_hazards else []),
        ))
    if flow_hazards:
        # With 'fused' the camera tracks carry their uncertainty and go to the fusion node.
        tracker_parameters = {'use_sim_time': True}
        if fused:
            tracker_parameters.update({'topic': '/obstacles/flow', 'uncertainty': True})
        nodes.append(Node(
            package='rapid_nav_perception',
            executable='flow_tracker',
            name='flow_tracker',
            output='screen',
            parameters=[tracker_parameters],
        ))
    if fused:
        nodes.append(Node(
            package='rapid_nav_perception',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ))

    rviz_config = os.path.join(share, 'rviz', 'sim.rviz')
    if planning or navigation:
        rviz_config = os.path.join(
            get_package_share_directory('rapid_nav_navigation'), 'rviz', 'navigation.rviz')
    if rviz and not headless:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
        ))
    return nodes
