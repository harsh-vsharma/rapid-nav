"""Test of experiment_report on a hand-made, incomplete experiment folder."""

import json
import os

from rapid_nav_tools import experiment_report as report
from rapid_nav_tools import experiments as ex


def make_run(directory, hazards, repeat, clearance, ok=True, valid=True):
    name = 'nav_run_crossing_dwb_{}_{}'.format(hazards, repeat)
    os.makedirs(os.path.join(directory, name))
    goals = [{'truth_success': ok, 'collision_episodes': 0 if clearance > 0 else 1,
              'min_clearance_dynamic_m': clearance, 'closest_approach_m': clearance + 0.3,
              'time_s': 20.0}]
    data = {'rtf': 1.0, 'goals': goals,
            'validity': [{'check': 'c', 'verdict': 'PASS' if valid else 'FAIL'}]}
    with open(os.path.join(directory, name, 'results.json'), 'w') as handle:
        json.dump(data, handle)
    with open(os.path.join(directory, name, 'environment.txt'), 'w') as handle:
        handle.write('ROS_DISTRO: jazzy\n')
    ex.append_index(os.path.join(directory, 'index.csv'),
                    {'scenario': 'crossing', 'controller': 'dwb', 'hazards': hazards,
                     'repeat': repeat, 'bundle': name, 'exit_code': 0, 'wall_s': 200,
                     'valid': int(valid)})


def test_report_on_an_incomplete_folder(tmp_path):
    directory = str(tmp_path)
    for repeat in range(1, 5):
        make_run(directory, 'off', repeat, 0.05 * repeat)
    for repeat in range(1, 4):
        make_run(directory, 'oracle', repeat, 0.4, valid=repeat != 2)
    with open(os.path.join(directory, 'matrix.yaml'), 'w') as handle:
        handle.write('name: t\nscenarios: [crossing]\ncontrollers: [dwb]\n'
                     'hazards: ["off", oracle]\nrepeats: 5\n')
    path = report.build_report(directory, directory)
    text = open(path).read()
    assert '10 planned' not in text and '10 runs planned, 3 not yet run' in text
    assert '| crossing | dwb | oracle | 2 | 1 |' in text
    assert 'too few pairs' in text
    assert 'ROS_DISTRO: jazzy' in text
    for name in ('cells.csv', 'pairs.csv', 'clearance_crossing.png', 'collisions_crossing.png'):
        assert os.path.getsize(os.path.join(directory, name)) > 0
