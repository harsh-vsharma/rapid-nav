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
    'world', 'headless', 'rviz', 'odom_tf', 'x', 'y', 'yaw', 'planning',
    'navigation', 'controller', 'scenario', 'rtf', 'hazards', 'perception', 'flow_backend',
    'flow_device',
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
