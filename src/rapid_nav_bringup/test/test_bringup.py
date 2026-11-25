"""Checks of the bringup package: the EKF configuration and the simulation launch file."""

import importlib.util
import os

from launch.actions import DeclareLaunchArgument
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.normpath(os.path.join(HERE, '..'))
EKF = os.path.join(PACKAGE, 'config', 'ekf.yaml')
SIM_LAUNCH = os.path.join(PACKAGE, 'launch', 'sim.launch.py')

EXPECTED_ARGUMENTS = {
    'stack', 'scan_filter', 'hazards', 'scenario', 'controller', 'governor', 'rtf', 'flow_backend',
    'headless', 'world', 'x', 'y', 'yaw', 'odom_tf',
}


def load_ekf_parameters():
    """Return the ros__parameters of ekf_filter_node."""
    with open(EKF, encoding='utf-8') as handle:
        document = yaml.safe_load(handle)
    return document['ekf_filter_node']['ros__parameters']


def test_ekf_config_loads_with_the_expected_frames():
    """ekf.yaml names the odom and base frames the TF tree uses."""
    parameters = load_ekf_parameters()
    assert parameters['odom_frame'] == 'odom'
    assert parameters['base_link_frame'] == 'base_footprint'
    assert parameters['publish_tf'] is True
    assert parameters['two_d_mode'] is True
    assert parameters['use_sim_time'] is True


def test_ekf_fusion_masks_are_fifteen_elements():
    """The wheels fuse vx and vy; the IMU fuses yaw rate and forward acceleration."""
    parameters = load_ekf_parameters()
    odom_config = parameters['odom0_config']
    imu_config = parameters['imu0_config']
    assert len(odom_config) == 15
    assert len(imu_config) == 15
    assert [i for i, flag in enumerate(odom_config) if flag] == [6, 7]
    assert [i for i, flag in enumerate(imu_config) if flag] == [11, 12]
    assert parameters['imu0_remove_gravitational_acceleration'] is True


def test_sim_launch_declares_the_arguments():
    """sim.launch.py declares the arguments of the simulation."""
    spec = importlib.util.spec_from_file_location('sim_launch', SIM_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    declared = {entity.name for entity in description.entities
                if isinstance(entity, DeclareLaunchArgument)}
    assert declared == EXPECTED_ARGUMENTS


def load_navigation_launch():
    """Import navigation.launch.py of the installed navigation package."""
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('rapid_nav_navigation')
    spec = importlib.util.spec_from_file_location(
        'navigation_launch', os.path.join(share, 'launch', 'navigation.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, os.path.join(share, 'config', 'navigation.yaml')


def written_config(hazards):
    """Return the YAML navigation_config writes for a hazards value."""
    module, config = load_navigation_launch()
    path = module.navigation_config(config, 'dwa', True, 'tree.xml', hazards)
    try:
        with open(path, encoding='utf-8') as handle:
            return yaml.safe_load(handle)
    finally:
        os.remove(path)


def test_hazards_off_leaves_the_parameters_of_the_plain_stack():
    """With hazards off the plugin lists are the old ones and no hazard block is written."""
    data = written_config('off')
    glob = data['global_costmap']['global_costmap']['ros__parameters']
    loc = data['local_costmap']['local_costmap']['ros__parameters']
    assert glob['plugins'] == ['static_layer', 'obstacle_layer', 'inflation_layer']
    assert loc['plugins'] == ['obstacle_layer', 'inflation_layer']
    assert 'hazard_layer' not in glob
    assert 'hazard_layer' not in loc


def test_hazards_oracle_adds_the_layer_after_inflation():
    """With a hazard source the layer is the last plugin of both costmaps, with its block."""
    for mode in ('oracle', 'oracle_static', 'flow'):
        data = written_config(mode)
        glob = data['global_costmap']['global_costmap']['ros__parameters']
        loc = data['local_costmap']['local_costmap']['ros__parameters']
        assert glob['plugins'] == [
            'static_layer', 'obstacle_layer', 'inflation_layer', 'hazard_layer']
        assert loc['plugins'] == ['obstacle_layer', 'inflation_layer', 'hazard_layer']
        for section in (glob, loc):
            block = section['hazard_layer']
            assert block['plugin'] == 'rapid_nav_hazard::HazardLayer'
            assert block['max_cost'] == 252
            assert block['horizon_s'] == 2.0


def test_hazards_rejects_other_values():
    """An unknown hazards value is an error."""
    module, config = load_navigation_launch()
    try:
        module.navigation_config(config, 'dwa', True, 'tree.xml', 'radar')
    except RuntimeError as error:
        assert 'hazards' in str(error)
    else:
        raise AssertionError('expected RuntimeError')


def load_sim_launch():
    """Import sim.launch.py of this package."""
    spec = importlib.util.spec_from_file_location('sim_launch', SIM_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def expect_error(function, *args, words=()):
    """Call the function, expect a RuntimeError whose message holds every word."""
    try:
        function(*args)
    except RuntimeError as error:
        for word in words:
            assert word in str(error)
    else:
        raise AssertionError('expected RuntimeError')


def test_arguments_accept_the_defaults_and_the_filter():
    """The defaults pass; other spawns off navigation; the filter with navigation and fused."""
    check = load_sim_launch().check_arguments
    check('sim', 'false', 'off', 'dwa', 'true', 'ekf', '0.0', '0.0', '0.0')
    check('planning', 'false', 'off', 'rpp', 'false', 'wheel', '1.0', '2.0', '0.5')
    check('navigation', 'false', 'flow', 'dwb', 'true', 'ekf', '0', '0', '0')
    check('navigation', 'true', 'fused', 'dwb', 'true', 'ekf', '0', '0', '0')


def test_arguments_reject_bad_values_and_combinations():
    """Reject unknown values, navigation off the default spawn, the filter without its inputs."""
    check = load_sim_launch().check_arguments
    ok = ['navigation', 'true', 'fused', 'dwb', 'true', 'ekf', '0', '0', '0']
    for index, bad, word in ((0, 'everything', 'stack'), (1, 'maybe', 'scan_filter'),
                             (2, 'radar', 'hazards'), (3, 'mpc', 'controller'),
                             (4, 'yes', 'governor'), (5, 'gps', 'odom_tf')):
        args = list(ok)
        args[index] = bad
        expect_error(check, *args, words=(word,))
    expect_error(check, 'navigation', 'false', 'off', 'dwb', 'true', 'ekf', '1.0', '0', '0',
                 words=('spawn',))
    expect_error(check, 'navigation', 'false', 'off', 'dwb', 'true', 'ekf', '0', '0', '0.5',
                 words=('spawn',))
    expect_error(check, 'navigation', 'true', 'flow', 'dwb', 'true', 'ekf', '0', '0', '0',
                 words=('hazards:=fused',))
    expect_error(check, 'planning', 'true', 'fused', 'dwb', 'true', 'ekf', '0', '0', '0',
                 words=('stack:=navigation',))


def test_slam_config_sets_the_scan_topic_and_the_clock():
    """slam_config writes the chosen scan topic and use_sim_time into slam_toolbox's block."""
    from ament_index_python.packages import get_package_share_directory
    module, _ = load_navigation_launch()
    share = get_package_share_directory('rapid_nav_navigation')
    for topic in ('/scan', '/scan_static'):
        path = module.slam_config(os.path.join(share, 'config', 'slam.yaml'), topic, True)
        try:
            with open(path, encoding='utf-8') as handle:
                params = yaml.safe_load(handle)['slam_toolbox']['ros__parameters']
        finally:
            os.remove(path)
        assert params['scan_topic'] == topic
        assert params['use_sim_time'] is True
        assert params['map_frame'] == 'map' and params['odom_frame'] == 'odom'
        assert params['base_frame'] == 'base_footprint'
        assert params['resolution'] == 0.05


def test_auto_arguments_give_the_demo_without_stack_and_the_old_defaults_with_it():
    """Without `stack` everything is on; with it the experiment defaults apply."""
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location('sim_launch', SIM_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    def context(**given):
        ctx = LaunchContext()
        values = {name: 'auto' for name in module.DEMO}
        values.update(given)
        ctx.launch_configurations.update(values)
        return ctx

    demo = context()
    assert [module.resolve(demo, n) for n in ('stack', 'hazards', 'scan_filter', 'scenario')] == \
        ['navigation', 'fused', 'true', 'crowd']
    assert module.resolve(context(scenario='crossing'), 'scenario') == 'crossing'
    old = context(stack='navigation')
    assert [module.resolve(old, n) for n in ('hazards', 'scan_filter', 'scenario',
                                             'flow_backend')] == \
        ['off', 'false', '', 'raft_small']


def test_slam_config_continues_a_saved_map_only_when_it_exists(tmp_path):
    """slam_map_file is used when <path>.posegraph exists, ignored otherwise."""
    module, _ = load_navigation_launch()
    from ament_index_python.packages import get_package_share_directory
    config = os.path.join(get_package_share_directory('rapid_nav_navigation'), 'config',
                          'slam.yaml')
    saved = str(tmp_path / 'demo_map')
    for exists in (False, True):
        if exists:
            (tmp_path / 'demo_map.posegraph').write_text('x')
        path = module.slam_config(config, '/scan', True, saved)
        try:
            with open(path, encoding='utf-8') as handle:
                params = yaml.safe_load(handle)['slam_toolbox']['ros__parameters']
        finally:
            os.remove(path)
        assert ('map_file_name' in params) == exists
        if exists:
            assert params['map_file_name'] == saved and params['map_start_at_dock'] is True
