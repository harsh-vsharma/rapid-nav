"""Experiment matrices: loading, expansion, bundle parsing, statistics and tables (no ROS).

A matrix names scenarios, controllers and hazard configurations and a number of repeats. The runner
(scripts/run_matrix.sh) executes every run of the expansion in a fresh simulation;
experiment_report reads the bundles and writes the tables. This module holds the parts that
need no simulation.
"""

import csv
import itertools
import json
import math
import os
import random
import sys

import numpy as np
import yaml

Z95 = 1.959963984540054
BOOTSTRAP_RESAMPLES = 10000
MIN_PAIRS = 3

INDEX_FIELDS = ['scenario', 'controller', 'hazards', 'repeat', 'bundle', 'exit_code', 'wall_s',
                'valid']

# The comparisons of the paired tables, per matrix kind: (A, B) reads "A against B".
DEFAULT_COMPARISONS = (
    ('oracle', 'off'),
    ('oracle_static', 'off'),
    ('oracle', 'oracle_static'),
    ('flow', 'off'),
    ('flow', 'oracle'),
)


# ---------------------------------------------------------------------------------------------
# Matrices
# ---------------------------------------------------------------------------------------------

def _word(value):
    """Return a matrix entry as text; YAML 1.1 reads an unquoted off/on as False/True."""
    if value is True:
        return 'on'
    if value is False:
        return 'off'
    return str(value)


def load_matrix(path):
    """Read a matrix file and return it as a dict with the keys filled in and checked."""
    with open(path, encoding='utf-8') as handle:
        raw = yaml.safe_load(handle)
    if not isinstance(raw, dict):
        raise ValueError('{}: a matrix file is a mapping'.format(path))
    for key in ('name', 'scenarios', 'controllers', 'hazards', 'repeats'):
        if key not in raw:
            raise ValueError('{}: missing key {}'.format(path, key))
    matrix = {
        'name': str(raw['name']),
        'rtf': float(raw.get('rtf', 1.0)),
        'scenarios': [str(s) for s in raw['scenarios']],
        'controllers': [str(c) for c in raw['controllers']],
        'hazards': [_word(h) for h in raw['hazards']],
        'repeats': int(raw['repeats']),
        'seed': int(raw.get('seed', 7)),
        'track_check': bool(raw.get('track_check', False)),
    }
    if matrix['repeats'] < 1:
        raise ValueError('{}: repeats must be at least 1'.format(path))
    for key in ('scenarios', 'controllers', 'hazards'):
        if not matrix[key] or len(set(matrix[key])) != len(matrix[key]):
            raise ValueError('{}: {} must be a list without repeats'.format(path, key))
    return matrix


def expand(matrix):
    """Return the ordered runs: repeats outer, the cells shuffled with the seed inside each repeat.

    One random generator, seeded once, shuffles the cells repeat by repeat, so a matrix with more
    repeats starts with the same order as one with fewer.
    """
    cells = list(itertools.product(matrix['scenarios'], matrix['controllers'], matrix['hazards']))
    rng = random.Random(matrix['seed'])
    runs = []
    for repeat in range(1, matrix['repeats'] + 1):
        order = list(cells)
        rng.shuffle(order)
        for scenario, controller, hazards in order:
            runs.append({
                'scenario': scenario, 'controller': controller, 'hazards': hazards,
                'repeat': repeat, 'rtf': matrix['rtf'],
                'track_check': bool(matrix['track_check'] and hazards == 'flow'),
            })
    return runs


def run_key(run):
    """Return the identity of a run: (scenario, controller, hazards, repeat)."""
    return (run['scenario'], run['controller'], run['hazards'], int(run['repeat']))


# ---------------------------------------------------------------------------------------------
# The index of finished runs
# ---------------------------------------------------------------------------------------------

def read_index(path):
    """Return the rows of an index.csv as dicts (an empty list if the file does not exist)."""
    if not os.path.isfile(path):
        return []
    with open(path, newline='', encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        row['repeat'] = int(row['repeat'])
    return rows


def append_index(path, row):
    """Append one row to index.csv, writing the header first when the file is new."""
    new = not os.path.isfile(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=INDEX_FIELDS)
        if new:
            writer.writeheader()
        writer.writerow({field: row.get(field, '') for field in INDEX_FIELDS})


def run_is_done(rows, run, directory=None):
    """Tell whether a run needs no further attempt: it has a good row, or it was tried twice.

    A good row has exit code 0 and a bundle (that exists in directory, when one is given).
    """
    key = run_key(run)
    mine = [r for r in rows if (r['scenario'], r['controller'], r['hazards'], r['repeat']) == key]
    for row in mine:
        bundle_ok = bool(row['bundle']) and (
            directory is None or os.path.isdir(os.path.join(directory, row['bundle'])))
        if str(row['exit_code']) == '0' and bundle_ok:
            return True
    return len(mine) >= 2


def pending(matrix, rows, directory=None):
    """Return the runs of the expansion that still need an attempt, in order."""
    return [run for run in expand(matrix) if not run_is_done(rows, run, directory)]


def best_rows(rows, directory=None):
    """Return {key: row}: per run the row to read (a good one, else the last that has a bundle)."""
    chosen = {}
    for row in rows:
        key = (row['scenario'], row['controller'], row['hazards'], row['repeat'])
        bundle_ok = bool(row['bundle']) and (
            directory is None or os.path.isdir(os.path.join(directory, row['bundle'])))
        good = str(row['exit_code']) == '0' and bundle_ok
        old = chosen.get(key)
        if old is None:
            chosen[key] = dict(row, _good=good, _has=bundle_ok)
        elif good or (not old['_good'] and (bundle_ok or not old['_has'])):
            chosen[key] = dict(row, _good=good, _has=bundle_ok)
    return chosen


# ---------------------------------------------------------------------------------------------
# Bundles
# ---------------------------------------------------------------------------------------------

def _results_path(bundle):
    """Return the nav_run results.json of a bundle (a track_check bundle keeps it in nav_run/)."""
    for candidate in (os.path.join(bundle, 'results.json'),
                      os.path.join(bundle, 'nav_run', 'results.json')):
        if os.path.isfile(candidate):
            try:
                with open(candidate, encoding='utf-8') as handle:
                    head = json.load(handle)
            except (OSError, ValueError):
                continue
            if isinstance(head, dict) and 'goals' in head:
                return candidate
    return None


def parse_bundle(path):
    """Return the per-run fields of a nav_run bundle.

    Keys: success, collisions, min_clearance, closest_approach, time, valid, reason (why it is
    invalid, else ''), goals, goals_reached, rtf. A bundle that lacks a needed field is invalid
    with the reason; its other fields are None.
    """
    out = {'success': None, 'collisions': None, 'min_clearance': None, 'closest_approach': None,
           'time': None, 'valid': False, 'reason': '', 'goals': 0, 'goals_reached': 0,
           'rtf': None}
    results = _results_path(path)
    if results is None:
        out['reason'] = 'no results.json with goals'
        return out
    with open(results, encoding='utf-8') as handle:
        data = json.load(handle)
    out['rtf'] = data.get('rtf')
    goals = data.get('goals')
    if not goals:
        out['reason'] = 'no goals'
        return out
    try:
        out['goals'] = len(goals)
        out['goals_reached'] = sum(1 for g in goals if g['truth_success'])
        out['success'] = out['goals_reached'] == out['goals']
        out['collisions'] = int(sum(g['collision_episodes'] for g in goals))
        clearances = [g['min_clearance_dynamic_m'] for g in goals
                      if g['min_clearance_dynamic_m'] is not None]
        approaches = [g['closest_approach_m'] for g in goals
                      if g['closest_approach_m'] is not None]
        out['min_clearance'] = float(min(clearances)) if clearances else None
        out['closest_approach'] = float(min(approaches)) if approaches else None
        out['time'] = float(np.mean([g['time_s'] for g in goals]))
    except (KeyError, TypeError) as error:
        out['success'] = None
        out['reason'] = 'missing field {}'.format(error)
        return out
    if out['min_clearance'] is None:
        out['reason'] = 'no dynamic clearance'
        return out
    bad = [row for row in data.get('validity', []) if row.get('verdict') != 'PASS']
    if bad:
        out['reason'] = 'validity: ' + '; '.join(
            '{} {}'.format(row.get('check'), row.get('verdict')) for row in bad)
        return out
    out['valid'] = True
    return out


def parse_index(directory):
    """Return one record per run of an experiments/<name>/ folder: index fields + parsed fields."""
    rows = read_index(os.path.join(directory, 'index.csv'))
    records = []
    for key, row in sorted(best_rows(rows, directory).items()):
        record = {'scenario': key[0], 'controller': key[1], 'hazards': key[2], 'repeat': key[3],
                  'bundle': row['bundle'], 'exit_code': row['exit_code'],
                  'wall_s': row.get('wall_s', '')}
        if row['bundle'] and os.path.isdir(os.path.join(directory, row['bundle'])):
            record.update(parse_bundle(os.path.join(directory, row['bundle'])))
        else:
            record.update({'success': None, 'collisions': None, 'min_clearance': None,
                           'closest_approach': None, 'time': None, 'valid': False,
                           'reason': 'no bundle (exit {})'.format(row['exit_code']), 'goals': 0,
                           'goals_reached': 0, 'rtf': None})
        records.append(record)
    return records


# ---------------------------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------------------------

def wilson(successes, n, z=Z95):
    """Return the Wilson score interval (low, high) of a proportion; (nan, nan) when n is 0."""
    if n <= 0:
        return (math.nan, math.nan)
    p = successes / n
    denominator = 1.0 + z * z / n
    centre = (p + z * z / (2.0 * n)) / denominator
    half = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / denominator
    return (max(0.0, centre - half), min(1.0, centre + half))


def bootstrap_mean(differences, seed=7, resamples=BOOTSTRAP_RESAMPLES):
    """Bootstrap the mean of paired differences: (mean, low, high) with a 95 % percentile interval.

    Fewer than MIN_PAIRS differences give None.
    """
    values = np.asarray(differences, dtype=np.float64)
    if values.size < MIN_PAIRS:
        return None
    rng = np.random.default_rng(seed)
    picks = rng.integers(0, values.size, size=(resamples, values.size))
    means = values[picks].mean(axis=1)
    low, high = np.percentile(means, [2.5, 97.5])
    return (float(values.mean()), float(low), float(high))


def _mean_sd(values):
    """Return (mean, sample standard deviation) of a list; None entries if empty."""
    if not values:
        return (None, None)
    mean = float(np.mean(values))
    sd = float(np.std(values, ddof=1)) if len(values) > 1 else 0.0
    return (mean, sd)


def cell_stats(records):
    """Return one dict per cell (scenario, controller, hazards) from the run records."""
    groups = {}
    for record in records:
        groups.setdefault((record['scenario'], record['controller'], record['hazards']),
                          []).append(record)
    cells = []
    for key in sorted(groups):
        runs = groups[key]
        valid = [r for r in runs if r['valid']]
        n = len(valid)
        wins = sum(1 for r in valid if r['success'])
        free = sum(1 for r in valid if r['collisions'] == 0)
        clearance = [r['min_clearance'] for r in valid]
        cells.append({
            'scenario': key[0], 'controller': key[1], 'hazards': key[2],
            'runs': len(runs), 'n_valid': n, 'excluded': len(runs) - n,
            'success': wins, 'success_rate': wins / n if n else None,
            'success_ci': wilson(wins, n),
            'collision_free': free, 'collision_free_rate': free / n if n else None,
            'collision_free_ci': wilson(free, n),
            'collisions_mean': _mean_sd([r['collisions'] for r in valid])[0],
            'collisions_sd': _mean_sd([r['collisions'] for r in valid])[1],
            'min_clearance_mean': _mean_sd(clearance)[0],
            'min_clearance_sd': _mean_sd(clearance)[1],
            'min_clearance_median': float(np.median(clearance)) if clearance else None,
            'time_mean': _mean_sd([r['time'] for r in valid])[0],
            'time_sd': _mean_sd([r['time'] for r in valid])[1],
        })
    return cells


def paired_differences(records, scenario, controller, a, b):
    """Return the paired differences A - B of one scenario and controller, matched by repeat.

    Only repeats where both runs are valid count. Returns (min clearance differences, collision
    differences, number of repeats).
    """
    by_repeat = {}
    for record in records:
        same = (record['scenario'], record['controller']) == (scenario, controller)
        if same and record['valid']:
            by_repeat.setdefault(record['hazards'], {})[record['repeat']] = record
    first = by_repeat.get(a, {})
    second = by_repeat.get(b, {})
    repeats = sorted(set(first) & set(second))
    clearance = [first[r]['min_clearance'] - second[r]['min_clearance'] for r in repeats]
    collisions = [first[r]['collisions'] - second[r]['collisions'] for r in repeats]
    return clearance, collisions, repeats


def available_comparisons(records, comparisons=DEFAULT_COMPARISONS):
    """Return the comparisons whose two configurations both occur in the records."""
    present = {r['hazards'] for r in records}
    return [(a, b) for a, b in comparisons if a in present and b in present]


def pair_stats(records, comparisons=None, seed=7):
    """Return the paired rows per scenario, controller and comparison, and the pooled rows.

    Each row: scenario ('all' when pooled), controller, a, b, n_pairs, and for min clearance and
    collisions a (mean, low, high) tuple or None when there are too few pairs.
    """
    comparisons = available_comparisons(records) if comparisons is None else comparisons
    scenarios = sorted({r['scenario'] for r in records})
    controllers = sorted({r['controller'] for r in records})
    rows = []
    pooled = []
    for controller in controllers:
        for a, b in comparisons:
            all_clearance = []
            all_collisions = []
            for scenario in scenarios:
                clearance, collisions, _ = paired_differences(records, scenario, controller, a, b)
                if not clearance:
                    continue
                all_clearance += clearance
                all_collisions += collisions
                rows.append({'scenario': scenario, 'controller': controller, 'a': a, 'b': b,
                             'n_pairs': len(clearance),
                             'clearance': bootstrap_mean(clearance, seed),
                             'collisions': bootstrap_mean(collisions, seed)})
            if all_clearance:
                pooled.append({'scenario': 'all', 'controller': controller, 'a': a, 'b': b,
                               'n_pairs': len(all_clearance),
                               'clearance': bootstrap_mean(all_clearance, seed),
                               'collisions': bootstrap_mean(all_collisions, seed)})
    return rows, pooled


def interval_words(interval):
    """Read a (mean, low, high) interval in words: excludes or includes zero, or too few pairs."""
    if interval is None:
        return 'too few pairs'
    _, low, high = interval
    if low > 0.0 or high < 0.0:
        return 'the interval excludes zero'
    return 'the interval includes zero'


# ---------------------------------------------------------------------------------------------
# Tables
# ---------------------------------------------------------------------------------------------

def _fmt(value, digits=2):
    """Format a number, or a dash for None and nan."""
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return '-'
    return '{:.{}f}'.format(value, digits)


def _rate(rate, interval):
    """Format a rate with its Wilson interval."""
    if rate is None:
        return '-'
    return '{:.2f} [{:.2f}, {:.2f}]'.format(rate, interval[0], interval[1])


def _interval(interval, digits=3):
    """Format a (mean, low, high) tuple."""
    if interval is None:
        return 'too few pairs'
    return '{:+.{d}f} [{:+.{d}f}, {:+.{d}f}]'.format(*interval, d=digits)


def cell_table_rows(cells):
    """Return (header, rows) of the per-cell table as lists of strings."""
    header = ['scenario', 'controller', 'hazards', 'n valid', 'excluded', 'success rate (95 % CI)',
              'collision-free (95 % CI)', 'collisions mean (sd)', 'min clearance m mean (sd)',
              'min clearance median', 'time s mean (sd)']
    rows = []
    for c in cells:
        rows.append([
            c['scenario'], c['controller'], c['hazards'], str(c['n_valid']), str(c['excluded']),
            _rate(c['success_rate'], c['success_ci']),
            _rate(c['collision_free_rate'], c['collision_free_ci']),
            '{} ({})'.format(_fmt(c['collisions_mean']), _fmt(c['collisions_sd'])),
            '{} ({})'.format(_fmt(c['min_clearance_mean'], 3), _fmt(c['min_clearance_sd'], 3)),
            _fmt(c['min_clearance_median'], 3),
            '{} ({})'.format(_fmt(c['time_mean'], 1), _fmt(c['time_sd'], 1))])
    return header, rows


def pair_table_rows(pairs):
    """Return (header, rows) of a paired table (per scenario or pooled)."""
    header = ['scenario', 'controller', 'A against B', 'pairs', 'min clearance A-B m (95 % CI)',
              'reading (clearance)', 'collisions A-B (95 % CI)',
              'reading (collisions)']
    rows = []
    for p in pairs:
        rows.append([p['scenario'], p['controller'], '{} vs {}'.format(p['a'], p['b']),
                     str(p['n_pairs']), _interval(p['clearance']), interval_words(p['clearance']),
                     _interval(p['collisions'], 2), interval_words(p['collisions'])])
    return header, rows


def markdown_table(header, rows):
    """Return a Markdown table as text."""
    lines = ['| ' + ' | '.join(header) + ' |', '|' + '---|' * len(header)]
    for row in rows:
        lines.append('| ' + ' | '.join(row) + ' |')
    return '\n'.join(lines) + '\n'


def write_csv(path, header, rows):
    """Write rows (lists) with a header to a CSV file."""
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)


# ---------------------------------------------------------------------------------------------
# Command line for the runner script
# ---------------------------------------------------------------------------------------------

def _runs_to_tsv(runs):
    """Print runs as tab-separated lines: scenario controller hazards repeat rtf track_check."""
    for run in runs:
        print('\t'.join([run['scenario'], run['controller'], run['hazards'], str(run['repeat']),
                         repr(run['rtf']), '1' if run['track_check'] else '0']))


def main(argv=None):
    """Run a helper command of scripts/run_matrix.sh: name, expand, pending, valid or append."""
    args = list(sys.argv[1:] if argv is None else argv)
    usage = ('usage: experiments name <matrix> | expand <matrix> | pending <matrix> <dir> | '
             'valid <bundle> | append <index.csv> <scenario> <controller> <hazards> <repeat> '
             '<bundle> <exit> <wall_s> <valid>')
    if not args:
        print(usage, file=sys.stderr)
        return 2
    command, rest = args[0], args[1:]
    if command == 'name' and len(rest) == 1:
        print(load_matrix(rest[0])['name'])
    elif command == 'expand' and len(rest) == 1:
        _runs_to_tsv(expand(load_matrix(rest[0])))
    elif command == 'pending' and len(rest) == 2:
        matrix = load_matrix(rest[0])
        _runs_to_tsv(pending(matrix, read_index(os.path.join(rest[1], 'index.csv')), rest[1]))
    elif command == 'valid' and len(rest) == 1:
        print('1' if parse_bundle(rest[0])['valid'] else '0')
    elif command == 'append' and len(rest) == 9:
        append_index(rest[0], dict(zip(INDEX_FIELDS, rest[1:])))
    else:
        print(usage, file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
