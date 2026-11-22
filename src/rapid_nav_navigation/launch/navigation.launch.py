"""Start the navigation stack: the map, AMCL, the planner, the controller, the behaviour server and
the BT navigator, under one lifecycle manager."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

BUILT_IN_MAPS = ('arena', 'maze')
CONTROLLERS = ('dwa', 'dwb', 'rpp')
HAZARDS = ('off', 'oracle', 'oracle_static', 'flow')


def generate_launch_description():
    """Declare the arguments; the nodes are built once their values are known."""
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='arena',
                              description="'arena', 'maze' or the path of a map YAML."),
        DeclareLaunchArgument('controller', default_value='dwa',
                              description="'dwa' (ours), 'dwb' (Nav2's DWB) or 'rpp' "
                                          "(Regulated Pure Pursuit)."),
        DeclareLaunchArgument('hazards', default_value='off',
                              description="'off', 'oracle', 'oracle_static' or 'flow': the hazard layer "
                                          'in both costmaps (the source is started by the '
                                          'scenario launch).'),
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
        return os.path.join(get_package_share_directory('rapid_nav_navigation'), 'maps',
                            name + '.yaml')
    return name


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
    With hazards 'oracle', 'oracle_static' or 'flow' the hazard layer joins both costmaps after the
    inflation layer; with 'off' its parameter blocks are removed, so the parameters are exactly
    those of a stack without the layer.
    """
    if hazards not in HAZARDS:
        raise RuntimeError("hazards must be 'off', 'oracle', 'oracle_static' or 'flow', got '{}'".format(
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


def launch_setup(context):
    """Resolve the arguments and build the nodes of the navigation stack."""
    share = get_package_share_directory('rapid_nav_navigation')
    controller = LaunchConfiguration('controller').perform(context).strip().lower()
    if controller not in CONTROLLERS:
        raise RuntimeError("controller must be 'dwa', 'dwb' or 'rpp', got '{}'".format(controller))
    hazards = LaunchConfiguration('hazards').perform(context).strip().lower()
    if hazards not in HAZARDS:
        raise RuntimeError("hazards must be 'off', 'oracle', 'oracle_static' or 'flow', got '{}'".format(
            hazards))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    config = navigation_config(
        os.path.join(share, 'config', 'navigation.yaml'), controller, use_sim_time,
        os.path.join(share, 'behavior_trees', 'navigate_rapid.xml'), hazards)
    map_yaml = map_yaml_path(LaunchConfiguration('map').perform(context))
    x = float(LaunchConfiguration('x').perform(context))
    y = float(LaunchConfiguration('y').perform(context))
    yaw = float(LaunchConfiguration('yaw').perform(context))

    managed = ['map_server', 'amcl', 'planner_server', 'controller_server', 'behavior_server',
               'bt_navigator']
    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[config, {'yaml_filename': map_yaml}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[config, {
                'initial_pose': {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw},
            }],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                         'node_names': managed}],
        ),
    ]
