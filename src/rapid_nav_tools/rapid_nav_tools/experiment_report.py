"""experiment_report: tables and plots of an experiments/<name>/ folder.

    experiment_report experiments/m1/ [--out <dir>]

Writes report.md, cells.csv, pairs.csv and the plots (Pillow, like the other tools) into --out
(default: the experiment folder itself). It handles incomplete matrices: a cell with fewer repeats
is reported with its n.
"""

import argparse
import math
import os
import sys

from PIL import Image, ImageDraw

from rapid_nav_tools import experiments as ex

HAZARD_ORDER = ('off', 'oracle_static', 'oracle', 'flow')
HAZARD_COLOURS = {'off': (90, 90, 90), 'oracle_static': (214, 140, 30), 'oracle': (30, 120, 200),
                  'flow': (40, 160, 90)}
WIDTH = 1000
HEIGHT = 520
MARGIN_LEFT = 70
MARGIN_RIGHT = 20
MARGIN_TOP = 40
MARGIN_BOTTOM = 90


def hazard_rank(name):
    """Return a sort key that puts the known configurations in their usual order."""
    return (HAZARD_ORDER.index(name) if name in HAZARD_ORDER else len(HAZARD_ORDER), name)


def nice_ticks(low, high, count=6):
    """Return round tick values covering [low, high]."""
    span = max(high - low, 1e-9)
    raw = span / count
    magnitude = 10 ** int(math.floor(math.log10(raw)))
    for factor in (1, 2, 2.5, 5, 10):
        step = factor * magnitude
        if step >= raw:
            break
    first = int(math.floor(low / step))
    ticks = []
    value = first * step
    while value <= high + step * 0.5:
        ticks.append(round(value, 10))
        value += step
    return ticks


def draw_strip(path, title, groups, ylabel, zero_line=False):
    """Draw a strip plot: one column per group, each valid run a dot, the mean a bar.

    groups: list of (label, colour, values). Empty groups are drawn as an empty column.
    """
    image = Image.new('RGB', (WIDTH, HEIGHT), 'white')
    draw = ImageDraw.Draw(image)
    values = [v for _, _, vs in groups for v in vs]
    low = min(values + ([0.0] if zero_line else []) + [0.0]) if values else 0.0
    high = max(values + [0.0]) if values else 1.0
    pad = 0.08 * max(high - low, 0.1)
    low, high = low - pad, high + pad
    plot_w = WIDTH - MARGIN_LEFT - MARGIN_RIGHT
    plot_h = HEIGHT - MARGIN_TOP - MARGIN_BOTTOM

    def y_of(v):
        return MARGIN_TOP + (high - v) / (high - low) * plot_h

    draw.text((MARGIN_LEFT, 12), title, fill='black')
    draw.rectangle([MARGIN_LEFT, MARGIN_TOP, WIDTH - MARGIN_RIGHT, MARGIN_TOP + plot_h],
                   outline=(120, 120, 120))
    for tick in nice_ticks(low, high):
        if low <= tick <= high:
            y = y_of(tick)
            draw.line([MARGIN_LEFT, y, WIDTH - MARGIN_RIGHT, y], fill=(228, 228, 228))
            draw.text((8, y - 5), '{:g}'.format(tick), fill='black')
    draw.text((8, MARGIN_TOP - 22), ylabel, fill='black')
    if zero_line and low < 0.0 < high:
        draw.line([MARGIN_LEFT, y_of(0.0), WIDTH - MARGIN_RIGHT, y_of(0.0)], fill=(200, 40, 40),
                  width=2)
    column = plot_w / max(len(groups), 1)
    for i, (label, colour, vs) in enumerate(groups):
        centre = MARGIN_LEFT + (i + 0.5) * column
        for j, v in enumerate(sorted(vs)):
            jitter = ((j % 5) - 2) * 6
            y = y_of(v)
            draw.ellipse([centre + jitter - 4, y - 4, centre + jitter + 4, y + 4], fill=colour)
        if vs:
            mean_y = y_of(sum(vs) / len(vs))
            draw.line([centre - 22, mean_y, centre + 22, mean_y], fill='black', width=3)
        for k, part in enumerate(label.split('\n')):
            draw.text((centre - 4 * len(part) / 1.2, MARGIN_TOP + plot_h + 8 + 12 * k), part,
                      fill='black')
        draw.text((centre - 12, MARGIN_TOP + plot_h + 8 + 12 * 3), 'n={}'.format(len(vs)),
                  fill=(90, 90, 90))
    image.save(path)


def plot_scenario(directory, scenario, records, metric, ylabel, zero_line, tag):
    """Write the strip plot of one metric of one scenario (columns: controller, configuration)."""
    controllers = sorted({r['controller'] for r in records})
    hazards = sorted({r['hazards'] for r in records}, key=hazard_rank)
    groups = []
    for controller in controllers:
        for hazard in hazards:
            vs = [r[metric] for r in records if r['valid'] and r['scenario'] == scenario
                  and r['controller'] == controller and r['hazards'] == hazard]
            groups.append(('{}\n{}'.format(controller, hazard),
                           HAZARD_COLOURS.get(hazard, (150, 50, 150)), vs))
    path = os.path.join(directory, '{}_{}.png'.format(tag, scenario))
    draw_strip(path, '{}: {} per run (dots), mean (bar)'.format(scenario, ylabel), groups, ylabel,
               zero_line)
    return os.path.basename(path)


def environment_lines(directory, records):
    """Return the environment.txt lines of the first run that has a bundle."""
    for record in records:
        if not record.get('bundle'):
            continue
        bundle = os.path.join(directory, record['bundle'])
        for candidate in (os.path.join(bundle, 'environment.txt'),
                          os.path.join(bundle, 'nav_run', 'environment.txt')):
            if os.path.isfile(candidate):
                with open(candidate, encoding='utf-8') as handle:
                    return [line.rstrip() for line in handle if line.strip()], record['bundle']
    return [], ''


def excluded_rows(records):
    """Return (header, rows) of the runs left out of the statistics, with the reason."""
    header = ['scenario', 'controller', 'hazards', 'repeat', 'bundle', 'exit', 'reason']
    rows = [[r['scenario'], r['controller'], r['hazards'], str(r['repeat']), r['bundle'] or '-',
             str(r['exit_code']), r['reason']] for r in records if not r['valid']]
    return header, rows


def build_report(directory, out):
    """Write report.md, cells.csv, pairs.csv and the plots; return the report's path."""
    records = ex.parse_index(directory)
    if not records:
        raise SystemExit('no runs in {}'.format(os.path.join(directory, 'index.csv')))
    matrix = None
    matrix_path = os.path.join(directory, 'matrix.yaml')
    if os.path.isfile(matrix_path):
        matrix = ex.load_matrix(matrix_path)
    os.makedirs(out, exist_ok=True)

    cells = ex.cell_stats(records)
    cell_header, cell_rows = ex.cell_table_rows(cells)
    ex.write_csv(os.path.join(out, 'cells.csv'), cell_header, cell_rows)
    pairs, pooled = ex.pair_stats(records, seed=matrix['seed'] if matrix else 7)
    pair_header, pair_rows = ex.pair_table_rows(pairs)
    _, pooled_rows = ex.pair_table_rows(pooled)
    ex.write_csv(os.path.join(out, 'pairs.csv'), pair_header, pair_rows + pooled_rows)
    excl_header, excl_rows = excluded_rows(records)

    valid = sum(1 for r in records if r['valid'])
    rtfs = [r['rtf'] for r in records if r.get('rtf')]
    env, env_bundle = environment_lines(directory, records)
    lines = ['# Experiment report: {}'.format(os.path.basename(os.path.normpath(directory))), '']
    lines.append('- runs recorded: {} ({} valid, {} excluded)'.format(
        len(records), valid, len(records) - valid))
    if matrix:
        total = len(ex.expand(matrix))
        lines.append('- matrix: {} runs planned, {} not yet run'.format(
            total, total - len(records)))
        lines.append('- target simulation speed (RTF): {}'.format(matrix['rtf']))
    if rtfs:
        lines.append('- measured simulation speed (RTF): mean {:.2f}, min {:.2f}, max {:.2f}'
                     .format(sum(rtfs) / len(rtfs), min(rtfs), max(rtfs)))
    lines += ['', 'Intervals: Wilson 95 % for rates; for paired differences (A minus B, matched '
              'by repeat) a bootstrap 95 % interval of the mean (10 000 resamples, percentile). '
              'No p-values: the reading says whether the interval excludes or includes zero.',
              '']
    lines += ['## Per cell', '', ex.markdown_table(cell_header, cell_rows)]
    lines += ['## Paired differences per scenario and controller', '',
              ex.markdown_table(pair_header, pair_rows) if pair_rows else 'No pairs.\n', '']
    lines += ['## Pooled over scenarios', '',
              ex.markdown_table(pair_header, pooled_rows) if pooled_rows else 'No pairs.\n', '']
    lines += ['## Excluded runs', '',
              ex.markdown_table(excl_header, excl_rows) if excl_rows else 'None.\n', '']
    lines += ['## Plots', '']
    for scenario in sorted({r['scenario'] for r in records}):
        clearance = plot_scenario(out, scenario, records, 'min_clearance',
                                  'min dynamic clearance (m)', True, 'clearance')
        collisions = plot_scenario(out, scenario, records, 'collisions', 'collision episodes',
                                   False, 'collisions')
        lines += ['### {}'.format(scenario), '', '![clearance]({})'.format(clearance), '',
                  '![collisions]({})'.format(collisions), '']
    if env:
        lines += ['## Environment of the first run ({})'.format(env_bundle), '', '```'] + env + [
            '```', '']
    path = os.path.join(out, 'report.md')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines))
    return path


def main(argv=None):
    """Run the command line; return the exit status."""
    parser = argparse.ArgumentParser(description='Tables and plots of an experiment folder.')
    parser.add_argument('directory', help='experiments/<name>/')
    parser.add_argument('--out', default=None,
                        help='output folder (default: the experiment folder)')
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    path = build_report(args.directory, args.out or args.directory)
    print('report: ' + path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
