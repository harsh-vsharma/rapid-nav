# Experiments

One run per cell says little: head-on and crossing encounters end
near zero clearance and differ from run to run. The experiment tools repeat every cell in a fresh simulation and report
the spread and the uncertainty.

## The matrices

Matrix files are in `src/rapid_nav_tools/config/experiments/`. A file names the scenarios, controllers, conditions
(the `hazards` list), the number of repeats, the simulation speed (`rtf`) and a seed. Every run uses the navigation
stack, which builds its map with SLAM.

| matrix | what | runs | speed |
|---|---|---|---|
| `m4` | the dynamic hybrid: `crossing`, `head_on`, `crowd` x `dwb` x `off`, `fused`, `fused_filter` x 3 repeats | 27 | RTF 0.25 |

A condition is a hazard source (`off`, `oracle`, `oracle_static`, `flow`, `fused`) or `fused_filter`: the fused tracks
with the dynamic scan filter in front of SLAM ([slam.md](slam.md)). Write a new matrix file for other combinations;
`track_check: true` adds the camera track check to every `flow` run.

Quote `off` in a matrix file (`"off"`): unquoted, YAML reads it as a boolean.

**Order.** The repeats are the outer loop. Inside each repeat the cells are shuffled with the file's seed, so a slow
drift of the machine (heat, other load) spreads over all configurations instead of favouring one. A matrix with more
repeats starts with the same order as one with fewer.

## Running

Docker must be running (`docker desktop start`). From the repository root:

```bash
scripts/dev.sh experiment m4      # or the path of a matrix file
```

Each run is one container with a fresh simulation (`nav-smoke`, or `track-smoke` for `flow` runs of a matrix with
`track_check: true`). The bundle is moved to `experiments/<name>/` (not tracked by git) and a line is added to
`experiments/<name>/index.csv`: scenario, controller, condition, repeat, bundle, exit code, wall time, valid.

- **Resume.** Run the same command again: runs with exit code 0 and a bundle are skipped.
- **Stop.** Ctrl+C, or `touch experiments/<name>/STOP`: the runner ends after the current run.
- **Failures.** A run that fails (non-zero exit, no bundle, or longer than three times its expected wall time) is
  retried once. A second failure stays in `index.csv` with its exit code and the runner moves on; the run is retried
  by a later restart only if it was tried less than twice.
- **Disk.** The runner stops cleanly when `/` has less than 3 GiB free (`MIN_FREE_GIB` changes the limit).

Time: one run takes about 2 to 7 minutes of wall time at RTF 1 in Docker on the development Mac (runs where the robot
waits or recovers are the longest), and about 1/RTF times that at a lower speed, so m4 needs about six hours. Natively the runs need the same simulated time; with a GPU and more cores they are faster.

## The report

```bash
uv run --with numpy --with pyyaml --with pillow \
    env PYTHONPATH=src/rapid_nav_tools python -m rapid_nav_tools.experiment_report experiments/m4/
```

or `ros2 run rapid_nav_tools experiment_report experiments/m4/` in the built workspace. It writes `report.md`,
`cells.csv`, `pairs.csv` and the plots next to the results (`--out` chooses another folder). It also works on a matrix
that is only partly done: each cell shows its own n.

**What a run is.** Success: every goal reached, on the simulator's ground truth. Collisions: the number of collision
episodes. Minimum clearance: the smallest distance between the robot's body and a moving obstacle over the run, in
metres; negative means overlap. Time: the mean time per goal. A run counts as *valid* only if the scenario played out as
scripted (the validity rows of the bundle all pass); invalid runs are left out of the statistics and listed per cell.

**Per cell.** Success rate and collision-free rate (runs with no collision) with Wilson 95 % intervals; mean and
standard deviation of collisions, minimum clearance and time; the median clearance.

**Paired differences.** Within a scenario and controller, two configurations A and B are compared run by run: the runs
with the same repeat number are a pair. The tables give the mean of A minus B for the minimum clearance (positive means
A kept more distance) and for collisions (negative means A had fewer), with a bootstrap 95 % interval of that mean
(10 000 resamples of the paired differences, percentile method, seed 7). With fewer than 3 valid pairs the table says
"too few pairs". The pooled table does the same over all scenarios of a controller.

There are no p-values. The tables read an interval in words: "the interval excludes zero" means the data point to a
difference in that direction, "includes zero" means they do not separate the two configurations. With fewer than 10 pairs a percentile bootstrap interval is too narrow; read
"excludes zero" on few pairs as a hint, not a finding (the tables mark such readings "(few pairs)"). With 5 repeats the
intervals are an account of the spread, not a test.

**SLAM.** A further table gives per cell the mean trajectory error (ATE RMSE), the map precision and recall, and the
ghost cells and the ghost cells left by moving obstacles ([slam.md](slam.md)).

**Plots.** Per scenario, the minimum clearance of every valid run (dots) and the mean (bar) per controller and
configuration, with a red line at zero, and the collision episodes in the same layout.
