"""
Start the RAPID-NAV demo: one command, everything on.

    ros2 launch rapid_nav_bringup sim.launch.py

Gazebo with the arena, the robot and a crowd of moving obstacles; SLAM builds the map online; Nav2
drives a looping patrol of numbered checkpoints (demo_mission); the camera's optical flow is fused
with the lidar into moving-obstacle tracks that the costmaps and the speed governor use, and the
scan filter keeps the movers out of the SLAM map. RViz shows the robot, the lidar outline, the
flow (mesh, one arrow per moving object), the checkpoints, the planned path and the controller's
local path, and where each tracked obstacle is predicted to be over the next 2 s (what the
planner avoids).

The other arguments are for the experiment scripts (scripts/dev.sh, scripts/run_matrix.sh) and
are not needed for the demo. Giving `stack` switches from the demo to the experiment defaults
(stack sim|perception|planning|navigation, hazards off, no scenario, no scan filter, RAFT flow),
which the scripts then set argument by argument.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

STACKS = ('sim', 'perception', 'planning', 'navigation')
HAZARDS = ('off', 'oracle', 'oracle_static', 'flow', 'fused')
CONTROLLERS = ('dwa', 'dwb', 'rpp')
ODOM_TF_MODES = ('ekf', 'wheel')
# What 'auto' means: everything on when `stack` is not given (the demo), the experiment defaults
# when it is (the scripts set each argument of the condition they run).
DEMO = {'stack': 'navigation', 'hazards': 'fused', 'scan_filter': 'true', 'scenario': 'crowd',
        'flow_backend': 'farneback'}
# The demo keeps its map: demo_mission saves the pose graph here, the next demo continues from it
# (delete the two demo_map.* files for a fresh map).
DEMO_MAP = os.path.join(os.path.expanduser('~'), '.ros', 'rapid_nav', 'demo_map')
EXPERIMENT = {'stack': 'sim', 'hazards': 'off', 'scan_filter': 'false', 'scenario': '',
              'flow_backend': 'raft_small'}


# Experiment arguments: (name, default, description). 'auto' means the demo value when `stack`
# is not given and the experiment default when it is (DEMO, EXPERIMENT).
EXPERIMENT_ARGUMENTS = (
    ('stack', 'auto', 'sim (robot only), perception (+ flow), planning (AMCL + the seven '
                      'planners) or navigation (SLAM + Nav2; needs the default spawn)'),
    ('hazards', 'auto', 'costmap hazards: off, oracle, oracle_static, flow or fused'),
    ('scan_filter', 'auto', 'with navigation and fused: SLAM maps the scan without the tracks'),
    ('scenario', 'auto', 'scenario of nav_scenarios.yaml whose obstacles to start (empty: none)'),
    ('controller', 'dwa', 'controller: dwa (ours), dwb or rpp'),
    ('governor', 'true', "with fused: the time-to-collision speed governor ('false': ablation)"),
    ('rtf', '1.0', 'target real-time factor (0.25 with the camera hazards on a CPU)'),
    ('flow_backend', 'auto', 'optical flow: farneback, raft_small, raft_large or frame_route'),
    ('x', '0.0', 'spawn x (m)'),
    ('y', '0.0', 'spawn y (m)'),
    ('yaw', '0.0', 'spawn heading (rad)'),
    ('odom_tf', 'ekf', "odom -> base_footprint from 'ekf' or 'wheel' (Gazebo DiffDrive)"),
)


def generate_launch_description():
    """Declare the arguments; the nodes are built once their values are known."""
    gazebo_share = get_package_share_directory('rapid_nav_gazebo')
    arguments = [
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run without the Gazebo window and without RViz.'),
        DeclareLaunchArgument('world',
                              default_value=os.path.join(gazebo_share, 'worlds', 'arena.sdf'),
                              description='(experiments) SDF world to load.'),
    ] + [DeclareLaunchArgument(name, default_value=default,
                               description='(experiments) ' + text)
         for name, default, text in EXPERIMENT_ARGUMENTS]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])


def resolve(context, name):
    """Return an argument's value as given, or for 'auto' the demo or the experiment default."""
    value = LaunchConfiguration(name).perform(context).strip()
    if value.lower() != 'auto':
        return value
    demo = LaunchConfiguration('stack').perform(context).strip().lower() == 'auto'
    return (DEMO if demo else EXPERIMENT)[name]


def word(context, name):
    """Return a launch argument's value (resolved), stripped and in lower case."""
    return (resolve(context, name) if name in DEMO
            else LaunchConfiguration(name).perform(context)).strip().lower()


def check_arguments(stack, scan_filter, hazards, controller, governor, odom_tf, x, y, yaw):
    """
    Check the arguments and how they combine; raise RuntimeError with the reason.

    The navigation stack runs SLAM, so it needs the default spawn (slam_toolbox's map frame starts
    at the robot; the scenarios' goals and the scoring are in the arena frame). The scan filter
    feeds that SLAM and removes the fused tracks, so it needs both.
    """
    for name, value, allowed in (('stack', stack, STACKS),
                                 ('scan_filter', scan_filter, ('true', 'false')),
                                 ('hazards', hazards, HAZARDS),
                                 ('controller', controller, CONTROLLERS),
                                 ('governor', governor, ('true', 'false')),
                                 ('odom_tf', odom_tf, ODOM_TF_MODES)):
        if value not in allowed:
            raise RuntimeError('{} must be one of {}, got {!r}'.format(
                name, ', '.join(allowed), value))
    if stack == 'navigation' and any(abs(float(v)) > 1e-9 for v in (x, y, yaw)):
        raise RuntimeError(
            'stack:=navigation needs the default spawn (x = y = yaw = 0): its SLAM starts the map '
            "frame at the robot and the scenarios' goals are in the arena frame; got x={} y={} "
            'yaw={}'.format(x, y, yaw))
    if scan_filter == 'true' and (stack != 'navigation' or hazards != 'fused'):
        raise RuntimeError(
            'scan_filter:=true needs stack:=navigation and hazards:=fused (it removes the fused '
            'tracks from the scan SLAM maps)')


def launch_setup(context):
    """Build the nodes for the resolved arguments."""
    share = get_package_share_directory('rapid_nav_bringup')
    gazebo_share = get_package_share_directory('rapid_nav_gazebo')
    navigation_share = get_package_share_directory('rapid_nav_navigation')

    stack, hazards = word(context, 'stack'), word(context, 'hazards')
    scan_filter = word(context, 'scan_filter')
    controller, governor = word(context, 'controller'), word(context, 'governor')
    odom_tf, headless = word(context, 'odom_tf'), word(context, 'headless') == 'true'
    x, y, yaw = (LaunchConfiguration(n).perform(context) for n in ('x', 'y', 'yaw'))
    check_arguments(stack, scan_filter, hazards, controller, governor, odom_tf, x, y, yaw)
    rtf = float(LaunchConfiguration('rtf').perform(context))
    if rtf <= 0.0:
        raise RuntimeError('rtf must be positive, got {}'.format(rtf))
    with_slam = stack == 'navigation'
    demo = LaunchConfiguration('stack').perform(context).strip().lower() == 'auto'
    scan_filter = scan_filter == 'true'

    nodes = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments=[('world', LaunchConfiguration('world').perform(context)),
                          ('headless', 'true' if headless else 'false'),
                          ('x', x), ('y', y), ('yaw', yaw)],
    )]
    if rtf != 1.0:
        # Wait for the world service of the running simulation, then set the target.
        nodes.append(ExecuteProcess(
            cmd=['bash', '-c',
                 'for i in $(seq 1 90); do gz service -s /world/arena/set_physics '
                 '--reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 2000 '
                 '--req "real_time_factor: {}" 2>/dev/null | grep -q true && exit 0; sleep 2; '
                 'done; echo "rtf: set_physics did not answer" >&2; exit 1'.format(rtf)],
            output='screen'))

    if stack == 'navigation':
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_share, 'launch', 'navigation.launch.py')),
            launch_arguments=[
                ('controller', controller), ('hazards', hazards), ('governor', governor),
                ('slam_scan_topic', '/scan_static' if scan_filter else '/scan'),
                ('use_sim_time', 'true'),
            ] + ([('slam_map_file', DEMO_MAP)] if demo else []),
        ))
    elif stack == 'planning':
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_share, 'launch', 'planning.launch.py')),
            launch_arguments=[('localization', 'amcl'), ('use_sim_time', 'true'),
                              ('x', x), ('y', y), ('yaw', yaw)],
        ))

    scenario = resolve(context, 'scenario')
    if scenario:
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'scenario.launch.py')),
            # In the demo the walkers make way for the robot, as people do.
            launch_arguments=[('scenario', scenario), ('hazards', hazards),
                              ('yield_to_robot', 'true' if demo else 'false')],
        ))

    if odom_tf == 'ekf':
        nodes.append(Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            output='screen', parameters=[os.path.join(share, 'config', 'ekf.yaml')],
        ))
    else:
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='gz_wheel_tf_bridge',
            output='screen',
            parameters=[{
                'config_file': os.path.join(gazebo_share, 'config', 'bridge_wheel_tf.yaml'),
                'use_sim_time': True,
            }],
        ))

    flow_hazards = hazards in ('flow', 'fused')
    fused = hazards == 'fused'
    # RViz shows the flow in every stack, so the flow node runs whenever RViz does.
    if stack == 'perception' or flow_hazards or not headless:
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rapid_nav_perception'), 'launch',
                'perception.launch.py')),
            # The flow picture is only needed by the perception stack (rqt); RViz uses markers.
            launch_arguments=[('backend', resolve(context, 'flow_backend')), ('device', 'auto')]
            + ([('num_threads', '4')] if flow_hazards else [])
            + ([] if stack == 'perception' else [('publish_vis', 'false')]),
        ))
    if flow_hazards:
        # With 'fused' the camera tracks carry their uncertainty and go to the fusion node.
        tracker_parameters = {'use_sim_time': True}
        if fused:
            tracker_parameters.update({'topic': '/obstacles/flow', 'uncertainty': True})
        if with_slam:
            # Its own TF node: /tf keeps arriving while the tracker waits for the newest stamp.
            tracker_parameters['tf_listener_node'] = 'dedicated'
        nodes.append(Node(
            package='rapid_nav_perception', executable='flow_tracker', name='flow_tracker',
            output='screen', parameters=[tracker_parameters],
        ))
    if fused:
        fusion_parameters = {'use_sim_time': True}
        if with_slam:
            # The map grows and may absorb a mover: unknown cells are static, tracks are guarded.
            fusion_parameters.update({'map_mode': 'slam', 'track_guard': True})
        nodes.append(Node(
            package='rapid_nav_perception', executable='fusion_node', name='fusion_node',
            output='screen', parameters=[fusion_parameters],
        ))
    if scan_filter:
        nodes.append(Node(
            package='rapid_nav_perception', executable='scan_filter_node', name='scan_filter',
            output='screen', parameters=[{'use_sim_time': True}],
        ))

    if demo:
        # The demo's task: patrol the checkpoints (its first goal also starts the crowd). It
        # saves the SLAM map after every checkpoint reached.
        os.makedirs(os.path.dirname(DEMO_MAP), exist_ok=True)
        nodes.append(Node(
            package='rapid_nav_tools', executable='demo_mission', name='demo_mission',
            output='screen', parameters=[{'use_sim_time': True, 'map_file': DEMO_MAP}],
        ))
    if not headless:
        rviz_config = os.path.join(share, 'rviz', 'sim.rviz')
        if stack in ('planning', 'navigation'):
            rviz_config = os.path.join(navigation_share, 'rviz', 'navigation.rviz')
        # The markers of the minimal view: lidar outline, flow instances, their mesh and motion.
        nodes.append(Node(
            package='rapid_nav_perception', executable='scene_viz', name='scene_viz',
            output='screen', parameters=[{'use_sim_time': True}],
        ))
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_config], parameters=[{'use_sim_time': True}],
        ))
    return nodes
