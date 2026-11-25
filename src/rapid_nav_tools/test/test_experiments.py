"""Tests of the experiment matrices and their statistics (pure Python, no ROS)."""

import json
import os
import tempfile

import pytest
from rapid_nav_tools import experiments as ex

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'experiments')

# Matrices of different shapes for the expansion and resume logic (not shipped).
FIXTURES = {
    'pilot': 'name: pilot\nrtf: 1.0\nscenarios: [crossing]\ncontrollers: [dwb]\n'
             'hazards: ["off", oracle]\nrepeats: 2\nseed: 7\n',
    'm1': 'name: m1\nrtf: 1.0\nscenarios: [crossing, head_on, crowd]\n'
          'controllers: [dwa, dwb, rpp]\nhazards: ["off", oracle_static, oracle]\nrepeats: 5\n'
          'seed: 7\n',
    'm2': 'name: m2\nrtf: 0.25\nscenarios: [crossing, head_on, crowd]\ncontrollers: [dwb]\n'
          'hazards: ["off", oracle, flow]\nrepeats: 5\nseed: 7\ntrack_check: true\n',
}


def fixture_path(name):
    """Return the path of a fixture matrix, written once to a temporary folder."""
    folder = os.path.join(tempfile.gettempdir(),
                          'rapid_nav_matrix_fixtures_{}'.format(os.getpid()))
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, name + '.yaml')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(FIXTURES[name])
    return path


def matrix(name):
    """Return a fixture matrix, or a shipped one (m4)."""
    if name in FIXTURES:
        return ex.load_matrix(fixture_path(name))
    return ex.load_matrix(os.path.join(CONFIG, name + '.yaml'))


def test_counts_of_the_matrices():
    assert len(ex.expand(matrix('m1'))) == 135
    assert len(ex.expand(matrix('m2'))) == 45
    assert len(ex.expand(matrix('pilot'))) == 4
    assert len(ex.expand(matrix('m4'))) == 27


def test_repeats_are_the_outer_loop_and_each_cell_once_per_repeat():
    m = matrix('m1')
    runs = ex.expand(m)
    cells = len(m['scenarios']) * len(m['controllers']) * len(m['hazards'])
    for repeat in range(1, m['repeats'] + 1):
        block = runs[(repeat - 1) * cells:repeat * cells]
        assert {r['repeat'] for r in block} == {repeat}
        keys = {(r['scenario'], r['controller'], r['hazards']) for r in block}
        assert len(keys) == cells


def test_same_seed_same_order_and_other_seed_other_order():
    m = matrix('m1')
    assert ex.expand(m) == ex.expand(dict(m))
    other = dict(m, seed=8)
    assert [ex.run_key(r) for r in ex.expand(other)] != [ex.run_key(r) for r in ex.expand(m)]


def test_cells_are_shuffled_inside_a_repeat():
    m = matrix('m1')
    first = [(r['scenario'], r['controller'], r['hazards']) for r in ex.expand(m)[:27]]
    assert first != sorted(first)


def test_more_repeats_keep_the_earlier_order():
    m = matrix('m1')
    short = ex.expand(dict(m, repeats=2))
    assert ex.expand(m)[:len(short)] == short


def test_unquoted_off_in_yaml_stays_the_word_off(tmp_path):
    path = tmp_path / 'm.yaml'
    path.write_text('name: x\nscenarios: [crossing]\ncontrollers: [dwb]\n'
                    'hazards: [off, oracle]\nrepeats: 1\n')
    assert ex.load_matrix(str(path))['hazards'] == ['off', 'oracle']
    assert matrix('pilot')['hazards'] == ['off', 'oracle']


def test_track_check_only_on_flow_runs_of_m2():
    assert all(r['track_check'] == (r['hazards'] == 'flow') for r in ex.expand(matrix('m2')))
    assert not any(r['track_check'] for r in ex.expand(matrix('m1')))
    assert matrix('m2')['rtf'] == 0.25


def test_pending_skips_good_rows_and_runs_tried_twice(tmp_path):
    m = matrix('pilot')
    runs = ex.expand(m)
    index = str(tmp_path / 'index.csv')
    (tmp_path / 'b1').mkdir()
    ex.append_index(index, dict(runs[0], bundle='b1', exit_code=0, wall_s=1, valid=1))
    ex.append_index(index, dict(runs[1], bundle='', exit_code=1, wall_s=1, valid=''))
    rows = ex.read_index(index)
    todo = ex.pending(m, rows, str(tmp_path))
    assert [ex.run_key(r) for r in todo] == [ex.run_key(r) for r in runs[1:]]
    ex.append_index(index, dict(runs[1], bundle='', exit_code=1, wall_s=1, valid=''))
    todo = ex.pending(m, ex.read_index(index), str(tmp_path))
    assert [ex.run_key(r) for r in todo] == [ex.run_key(r) for r in runs[2:]]


def test_a_good_row_whose_bundle_is_gone_is_pending(tmp_path):
    m = matrix('pilot')
    runs = ex.expand(m)
    index = str(tmp_path / 'index.csv')
    ex.append_index(index, dict(runs[0], bundle='missing', exit_code=0, wall_s=1, valid=1))
    todo = ex.pending(m, ex.read_index(index), str(tmp_path))
    assert len(todo) == 4


def test_command_line_append_and_pending(tmp_path, capsys):
    path = fixture_path('pilot')
    index = str(tmp_path / 'index.csv')
    assert ex.main(['append', index, 'crossing', 'dwb', 'off', '1', 'b1', '0', '115', '1']) == 0
    (tmp_path / 'b1').mkdir()
    assert ex.read_index(index)[0]['wall_s'] == '115'
    assert ex.main(['pending', path, str(tmp_path)]) == 0
    lines = capsys.readouterr().out.strip().splitlines()
    assert len(lines) == 3 and not any(
        line.startswith('crossing\tdwb\toff\t1\t') for line in lines)


# --- statistics -------------------------------------------------------------------------------

def test_wilson_known_values():
    low, high = ex.wilson(4, 5)
    assert abs(low - 0.376) < 0.001 and abs(high - 0.964) < 0.001
    assert ex.wilson(0, 0)[0] != ex.wilson(0, 0)[0]
    low, high = ex.wilson(5, 5)
    assert abs(low - 0.566) < 0.001 and high == 1.0


def test_bootstrap_of_a_constant_difference_has_zero_width():
    mean, low, high = ex.bootstrap_mean([0.25] * 5)
    assert mean == low == high == 0.25


def test_bootstrap_is_seeded_and_brackets_the_mean():
    values = [0.1, 0.4, -0.2, 0.3, 0.5, 0.0]
    first = ex.bootstrap_mean(values)
    assert first == ex.bootstrap_mean(values)
    assert first[1] <= first[0] <= first[2]


def test_too_few_pairs():
    assert ex.bootstrap_mean([1.0, 2.0]) is None
    assert ex.interval_words(None) == 'too few pairs'
    assert ex.interval_words((0.2, 0.1, 0.3)) == 'the interval excludes zero'
    assert ex.interval_words((0.0, -0.1, 0.1)) == 'the interval includes zero'
    assert ex.interval_words((0.2, 0.1, 0.3), 9) == 'the interval excludes zero (few pairs)'
    assert ex.interval_words((0.0, -0.1, 0.1), 3) == 'the interval includes zero (few pairs)'
    assert ex.interval_words((0.2, 0.1, 0.3), 10) == 'the interval excludes zero'


def write_bundle(path, goals, validity_ok=True, nested=False):
    os.makedirs(str(path), exist_ok=True)
    target = path / 'nav_run' if nested else path
    os.makedirs(str(target), exist_ok=True)
    data = {'rtf': 1.0, 'goals': goals,
            'validity': [{'check': 'walker_1 started',
                          'verdict': 'PASS' if validity_ok else 'FAIL'}]}
    with open(str(target / 'results.json'), 'w') as handle:
        json.dump(data, handle)


def goal(ok=True, collisions=0, clearance=0.4, approach=0.6, time=20.0):
    return {'truth_success': ok, 'collision_episodes': collisions,
            'min_clearance_dynamic_m': clearance, 'closest_approach_m': approach, 'time_s': time}


def test_parse_bundle_fields(tmp_path):
    write_bundle(tmp_path / 'b', [goal(), goal(collisions=1, clearance=-0.05, time=30.0)])
    run = ex.parse_bundle(str(tmp_path / 'b'))
    assert run['valid'] and run['success']
    assert run['collisions'] == 1 and run['goals_reached'] == 2
    assert abs(run['min_clearance'] + 0.05) < 1e-9 and run['time'] == 25.0


def test_parse_bundle_of_a_track_check_bundle(tmp_path):
    write_bundle(tmp_path / 'b', [goal()], nested=True)
    assert ex.parse_bundle(str(tmp_path / 'b'))['valid']


def test_invalid_validity_and_missing_field_are_reported(tmp_path):
    write_bundle(tmp_path / 'bad', [goal()], validity_ok=False)
    run = ex.parse_bundle(str(tmp_path / 'bad'))
    assert not run['valid'] and 'validity' in run['reason']
    write_bundle(tmp_path / 'old', [{'truth_success': True, 'time_s': 5.0}])
    run = ex.parse_bundle(str(tmp_path / 'old'))
    assert not run['valid'] and 'missing field' in run['reason']
    assert not ex.parse_bundle(str(tmp_path / 'nothing'))['valid']


def test_parse_a_real_like_bundle(tmp_path):
    here = os.path.join(os.path.dirname(__file__), 'data', 'experiments', 'real_like.json')
    os.makedirs(str(tmp_path / 'b'))
    with open(here) as source, open(str(tmp_path / 'b' / 'results.json'), 'w') as target:
        target.write(source.read())
    run = ex.parse_bundle(str(tmp_path / 'b'))
    assert run['valid'] and run['success'] and run['collisions'] == 1
    assert abs(run['min_clearance'] + 0.058087) < 1e-4


def record(hazards, repeat, valid=True, clearance=0.3, collisions=0, success=True,
           scenario='crossing', controller='dwb'):
    return {'scenario': scenario, 'controller': controller, 'hazards': hazards, 'repeat': repeat,
            'valid': valid, 'min_clearance': clearance, 'collisions': collisions,
            'success': success, 'time': 20.0, 'closest_approach': 0.5}


def test_invalid_runs_are_excluded_and_counted():
    records = [record('off', 1), record('off', 2), record('off', 3, valid=False),
               record('off', 4, clearance=0.1, success=False)]
    cell = ex.cell_stats(records)[0]
    assert cell['runs'] == 4 and cell['n_valid'] == 3 and cell['excluded'] == 1
    assert cell['success'] == 2
    assert abs(cell['min_clearance_mean'] - (0.3 + 0.3 + 0.1) / 3) < 1e-9
    assert cell['min_clearance_median'] == 0.3


def test_pairs_match_by_repeat_and_drop_invalid_partners():
    records = []
    for repeat in range(1, 6):
        records.append(record('off', repeat, clearance=0.1))
        records.append(record('oracle', repeat, clearance=0.4, valid=repeat != 3))
    clearance, collisions, repeats = ex.paired_differences(
        records, 'crossing', 'dwb', 'oracle', 'off')
    assert repeats == [1, 2, 4, 5]
    assert all(abs(d - 0.3) < 1e-9 for d in clearance) and collisions == [0, 0, 0, 0]
    rows, pooled = ex.pair_stats(records)
    assert rows[0]['n_pairs'] == 4 and pooled[0]['n_pairs'] == 4
    mean, low, high = rows[0]['clearance']
    assert abs(mean - 0.3) < 1e-9 and abs(high - low) < 1e-9


def test_too_few_pairs_in_a_cell_but_pooled_has_enough():
    records = []
    for scenario in ('crossing', 'head_on'):
        for repeat in (1, 2):
            records.append(record('off', repeat, scenario=scenario))
            records.append(record('oracle', repeat, clearance=0.5, scenario=scenario))
    rows, pooled = ex.pair_stats(records)
    assert all(r['clearance'] is None for r in rows)
    assert pooled[0]['n_pairs'] == 4 and pooled[0]['clearance'] is not None


def test_tables_are_markdown():
    cells = ex.cell_stats([record('off', 1), record('off', 2)])
    header, rows = ex.cell_table_rows(cells)
    text = ex.markdown_table(header, rows)
    assert text.startswith('| scenario') and '1.00 [0.34, 1.00]' in text


def test_condition_words_map_to_hazards_and_filter():
    assert ex.condition('off') == ('off', False)
    assert ex.condition('fused') == ('fused', False)
    assert ex.condition('fused_filter') == ('fused', True)
    for bad in ('radar', 'flow_filter', 'slam_fused', 'off_filter'):
        with pytest.raises(ValueError):
            ex.condition(bad)


def test_condition_word_is_the_inverse():
    for word in ('off', 'oracle', 'oracle_static', 'flow', 'fused', 'fused_filter'):
        assert ex.condition_word(*ex.condition(word)) == word
    with pytest.raises(ValueError):
        ex.condition_word('flow', True)


def test_nav_args_of_the_conditions():
    assert ex.nav_args('fused') == ['fused']
    assert ex.nav_args('off') == ['off']
    assert ex.nav_args('fused_filter') == ['fused', '--scan-filter']


def test_m4_matrix_loads_and_expands_to_27_runs():
    m = matrix('m4')
    assert m['hazards'] == ['off', 'fused', 'fused_filter']
    assert len(ex.expand(m)) == 27 and m['rtf'] == 0.25
    assert ('fused_filter', 'fused') in ex.DEFAULT_COMPARISONS


def test_load_matrix_rejects_an_unknown_condition(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('name: bad\nscenarios: [crossing]\ncontrollers: [dwb]\n'
                    'hazards: [radar]\nrepeats: 1\n')
    with pytest.raises(ValueError):
        ex.load_matrix(str(path))


def test_parse_bundle_reads_the_slam_fields(tmp_path):
    goal = {'truth_success': True, 'collision_episodes': 0, 'min_clearance_dynamic_m': 0.2,
            'closest_approach_m': 0.6, 'time_s': 10.0}
    data = {'goals': [goal], 'validity': [], 'rtf': 0.25,
            'slam': {'trajectory': {'ate_rmse_m': 0.04},
                     'map_quality': {'aligned': {'precision': 0.9, 'recall': 0.8, 'f1': 0.85,
                                                 'ghost_cells': 12, 'mover_ghost_cells': 7}}}}
    (tmp_path / 'results.json').write_text(json.dumps(data))
    out = ex.parse_bundle(str(tmp_path))
    assert out['valid'] and out['slam_ate_m'] == 0.04
    assert out['map_precision'] == 0.9 and out['mover_ghost_cells'] == 7
    del data['slam']
    (tmp_path / 'results.json').write_text(json.dumps(data))
    assert ex.parse_bundle(str(tmp_path))['map_precision'] is None
