"""
Helpers shared by the check tools: the bundle folder and the report table.

Nothing here talks to ROS; the tools own their nodes and call these functions to write results.
"""

import datetime
import json
import os
import platform
import shutil
import subprocess


def stamp_seconds(stamp):
    """Return a builtin_interfaces stamp in seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def verdict_of(ok):
    """Return PASS when ok, else FAIL."""
    return 'PASS' if ok else 'FAIL'


def format_number(value, digits=3):
    """Format a number for the table, or a dash when it is missing."""
    if value is None:
        return '-'
    return '{:.{}f}'.format(value, digits)


def run_text(command):
    """Run a command and return its first output line, or a short note when it cannot run."""
    if shutil.which(command[0]) is None:
        return 'not available'
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=20, check=False)
    except (OSError, subprocess.SubprocessError) as error:
        return 'error: {}'.format(error)
    lines = (result.stdout or result.stderr).strip().splitlines()
    if result.returncode != 0 or not lines:
        return 'not available'
    return lines[0]


def ram_text():
    """Return the total RAM from /proc/meminfo, or a note when it is not available."""
    try:
        with open('/proc/meminfo', encoding='utf-8') as handle:
            for line in handle:
                if line.startswith('MemTotal:'):
                    gibibytes = float(line.split()[1]) / 1024.0 / 1024.0
                    return '{:.1f} GiB'.format(gibibytes)
    except OSError:
        pass
    return 'not available'


def git_text():
    """Return the short commit and the branch when the current folder is in the repository."""
    commit = run_text(['git', 'rev-parse', '--short', 'HEAD'])
    if commit == 'not available':
        return 'not in a git repository'
    return '{} on {}'.format(commit, run_text(['git', 'branch', '--show-current']))


def environment_lines(argv):
    """Return the lines of environment.txt."""
    return [
        'date: ' + datetime.datetime.now().isoformat(timespec='seconds'),
        'ROS_DISTRO: ' + os.environ.get('ROS_DISTRO', 'unset'),
        'gz sim: ' + run_text(['gz', 'sim', '--versions']),
        'platform: ' + platform.platform(),
        'CPUs: {}'.format(os.cpu_count()),
        'RAM: ' + ram_text(),
        'GPU: ' + run_text(['nvidia-smi', '--query-gpu=name,memory.total',
                            '--format=csv,noheader']),
        'git: ' + git_text(),
        'arguments: ' + ' '.join(argv),
    ]


class Report:
    """
    Collect the check rows.

    The verdicts are PASS, WARN, FAIL and INFO. INFO rows report a measured value and never fail a
    run.
    """

    def __init__(self):
        """Start with no rows."""
        self.rows = []

    def add(self, name, value, expected, verdict):
        """Add one check row."""
        self.rows.append({'check': name, 'value': value, 'expected': expected, 'verdict': verdict})

    def count(self, verdict):
        """Return how many rows have the given verdict."""
        return sum(1 for row in self.rows if row['verdict'] == verdict)


def summary_text(title, environment, rtf, rows):
    """Return the Markdown summary: title, environment, real-time factor and the check table."""
    lines = ['# {}'.format(title), '']
    lines += ['- ' + line for line in environment]
    lines += ['- real-time factor: {:.3f}'.format(rtf), '']
    lines += ['| check | value | expected | verdict |', '|---|---|---|---|']
    for row in rows:
        lines.append('| {} | {} | {} | {} |'.format(
            row['check'], row['value'], row['expected'], row['verdict']))
    failures = sum(1 for row in rows if row['verdict'] == 'FAIL')
    warnings = sum(1 for row in rows if row['verdict'] == 'WARN')
    lines += ['', 'Result: {} failed, {} warnings, {} checks.'.format(
        failures, warnings, len(rows)), '']
    return '\n'.join(lines)


def write_outputs(out_dir, title, environment, rtf, rows, rates, values):
    """Write environment.txt, results.json and summary.md into the bundle folder."""
    with open(os.path.join(out_dir, 'environment.txt'), 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(environment) + '\n')
    results = {
        'environment': environment,
        'real_time_factor': rtf,
        'rates': rates,
        'values': values,
        'checks': rows,
    }
    with open(os.path.join(out_dir, 'results.json'), 'w', encoding='utf-8') as handle:
        json.dump(results, handle, indent=2, default=float)
    with open(os.path.join(out_dir, 'summary.md'), 'w', encoding='utf-8') as handle:
        handle.write(summary_text(title, environment, rtf, rows))
