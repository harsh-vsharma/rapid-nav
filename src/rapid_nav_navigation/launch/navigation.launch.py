"""
Start the navigation stack: SLAM, the planner, the controller, the behaviour server and the BT
navigator.

slam_toolbox (online async) builds the map from the lidar while the robot drives and publishes
/map and map -> odom; there is no pre-built map and no AMCL. It configures and activates itself,
as in slam_toolbox's own launch, outside the lifecycle manager, which runs the Nav2 servers.
The map frame starts at the robot, so the simulation must spawn it at the arena origin
(sim.launch.py checks that).
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import yaml

CONTROLLERS = ('dwa', 'dwb', 'rpp')
HAZARDS = ('off', 'oracle', 'oracle_static', 'flow', 'fused')


def generate_launch_description():
    """Declare the arguments; the nodes are built once their values are known."""
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='dwa',
                              description="'dwa' (ours), 'dwb' (Nav2's DWB) or 'rpp' "
                                          "(Regulated Pure Pursuit)."),
        DeclareLaunchArgument('hazards', default_value='off',
                              description="'off', 'oracle', 'oracle_static', 'flow' or "
                                          "'fused': the hazard layer in both costmaps (the "
                                          'source is started by the simulation launch); '
                                          "'fused' also starts the speed governor."),
        DeclareLaunchArgument('governor', default_value='true',
                              description="With hazards 'fused': start the time-to-collision "
                                          "speed governor ('true') or not ('false', the "
                                          'ablation).'),
        DeclareLaunchArgument('slam_scan_topic', default_value='/scan',
                              description="The scan slam_toolbox maps ('/scan_static' with the "
                                          'dynamic scan filter).'),
        DeclareLaunchArgument('slam_map_file', default_value='',
                              description='slam_toolbox pose graph to continue from (path '
                                          'without extension; ignored while <path>.posegraph '
                                          'does not exist). Empty: a new map.'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the simulated clock.'),
        OpaqueFunction(function=launch_setup),
    ])


def set_clock(section, use_sim_time):
    """Set use_sim_time in every node of a parameter section, the costmaps' nested nodes included."""
    for key, value in section.items():
        if not isinstance(value, dict):
            continue
        if key == 'ros__parameters':
            value['use_sim_time'] = use_sim_time
        else:
            set_clock(value, use_sim_time)


def navigation_config(config_path, controller, use_sim_time, bt_xml, hazards='off'):
    """
    Return a copy of the navigation YAML for this launch.

    The controller server runs one controller, named FollowPath, so the chosen controller's block
    is copied there. The clock is set on every node, because the costmaps run in nodes of their own
    that the launch's use_sim_time does not reach. The BT navigator gets the installed tree.
    With hazards 'oracle', 'oracle_static', 'flow' or 'fused' the hazard layer joins both costmaps
    after the inflation layer; with 'off' its parameter blocks are removed, so the parameters are
    exactly those of a stack without the layer.
    """
    if hazards not in HAZARDS:
        raise RuntimeError(
            "hazards must be 'off', 'oracle', 'oracle_static', 'flow' or 'fused', got '{}'".format(
                hazards))
    with open(config_path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    controller_params = data['controller_server']['ros__parameters']
    controller_params['FollowPath'] = dict(controller_params['FollowPath_' + controller])
    for name in ('dwa', 'dwb', 'rpp'):
        controller_params.pop('FollowPath_' + name, None)
    data['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = bt_xml
    for costmap in ('global_costmap', 'local_costmap'):
        section = data[costmap][costmap]['ros__parameters']
        if hazards == 'off':
            section.pop('hazard_layer', None)
        else:
            section['plugins'] = list(section['plugins']) + ['hazard_layer']
    set_clock(data, use_sim_time)
    descriptor, path = tempfile.mkstemp(prefix='navigation_', suffix='.yaml')
    with os.fdopen(descriptor, 'w', encoding='utf-8') as handle:
        yaml.safe_dump(data, handle)
    return path


def slam_config(config_path, scan_topic, use_sim_time, map_file=''):
    """
    Return a copy of the slam_toolbox YAML with this launch's scan topic and clock.

    With map_file (a path without extension) whose .posegraph exists, slam_toolbox loads that
    pose graph and keeps mapping from it, starting at its first pose (the spawn: the map frame
    of the saved map starts there too).
    """
    with open(config_path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    params = data['slam_toolbox']['ros__parameters']
    params['scan_topic'] = scan_topic
    params['use_sim_time'] = use_sim_time
    if map_file and os.path.isfile(map_file + '.posegraph'):
        params['map_file_name'] = map_file
        params['map_start_at_dock'] = True
    descriptor, path = tempfile.mkstemp(prefix='slam_', suffix='.yaml')
    with os.fdopen(descriptor, 'w', encoding='utf-8') as handle:
        yaml.safe_dump(data, handle)
    return path


def slam_nodes(config):
    """Return slam_toolbox as a lifecycle node that configures and then activates itself."""
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[config, {'use_lifecycle_manager': False}],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda action: action is slam,
        transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda action: action is slam,
            transition_id=Transition.TRANSITION_ACTIVATE))]))
    return [slam, activate, configure]


def launch_setup(context):
    """Resolve the arguments and build the nodes of the navigation stack."""
    share = get_package_share_directory('rapid_nav_navigation')
    controller = LaunchConfiguration('controller').perform(context).strip().lower()
    if controller not in CONTROLLERS:
        raise RuntimeError("controller must be 'dwa', 'dwb' or 'rpp', got '{}'".format(controller))
    hazards = LaunchConfiguration('hazards').perform(context).strip().lower()
    if hazards not in HAZARDS:
        raise RuntimeError(
            "hazards must be 'off', 'oracle', 'oracle_static', 'flow' or 'fused', got '{}'".format(
                hazards))
    governor = LaunchConfiguration('governor').perform(context).strip().lower()
    if governor not in ('true', 'false'):
        raise RuntimeError("governor must be 'true' or 'false', got '{}'".format(governor))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    config = navigation_config(
        os.path.join(share, 'config', 'navigation.yaml'), controller, use_sim_time,
        os.path.join(share, 'behavior_trees', 'navigate_rapid.xml'), hazards)
    scan_topic = LaunchConfiguration('slam_scan_topic').perform(context).strip()
    map_file = os.path.expanduser(LaunchConfiguration('slam_map_file').perform(context).strip())
    nodes = slam_nodes(slam_config(os.path.join(share, 'config', 'slam.yaml'), scan_topic,
                                   use_sim_time, map_file))
    if hazards == 'fused' and governor == 'true':
        # Reactive layer on top of the plan: a speed limit from the time to collision.
        nodes.append(Node(
            package='rapid_nav_navigation', executable='ttc_governor', name='ttc_governor',
            output='screen', parameters=[{'use_sim_time': use_sim_time}],
        ))
    servers = [('nav2_planner', 'planner_server'), ('nav2_controller', 'controller_server'),
               ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator')]
    for package, name in servers:
        nodes.append(Node(package=package, executable=name, name=name, output='screen',
                          parameters=[config]))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': [name for _, name in servers]}],
    ))
    return nodes
