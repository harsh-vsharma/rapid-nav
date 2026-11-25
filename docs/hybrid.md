# The hybrid: fused tracks, risk cost and a speed governor

`hazards:=fused` replaces "pick one velocity source" with a hybrid in which the reliability of what the robot sees
changes the behaviour continuously, in three places:

1. **Fusion.** The lidar and the camera both update one Kalman filter per obstacle, each weighted by its covariance.
   There is no switch between them: a confident camera track corrects the velocity fast, a doubtful one barely
   moves it, and the lidar keeps the position honest either way.
2. **Risk cost.** Each predicted position carries its uncertainty. The hazard layer turns it into a probability that
   the cell is occupied, so a confident track paints a sharp, high cost and an uncertain one a wide, low cost.
3. **Speed governor.** On top of Nav2's plan (deliberative), a time-to-collision check limits the speed (reactive):
   the closer and the more uncertain an approaching obstacle, the slower the robot may drive.

`hazards:=off|oracle|oracle_static|flow` are unchanged.

## 1. Camera reliability and lidar–camera fusion

**Camera reliability.** For every flow frame `flow_tracker` computes a reliability `r` in (0, 1]: a factor for the
network that ran (RAFT and SEA-RAFT 1.0, FlowNetS 0.6, Farnebäck 0.4; `FlowFrame.backend`, so `frame_route` is judged
frame by frame) times the share of samples that passed the texture test. Each camera track then gets

- `velocity_sigma = max(0.05, s_v / sqrt(n)) / r`
- `position_sigma = max(0.05, s_p)`
- `confidence = r`

where `s_v` and `s_p` are the spreads (root mean square of the per-axis standard deviations) of the velocities and
positions of the cluster's `n` samples. The tracker does this only with its parameter `uncertainty` true, which the
fused launch sets; it then publishes on `/obstacles/flow`.

**Lidar clusters.** `fusion_node` moves each `/scan` into `map` with TF at the scan stamp, drops the points within
0.15 m of an occupied cell of the static map (`/map`), links the rest when closer than 0.15 m and keeps groups of at
least 3 points. A cluster gives a centre (mean) and a radius.

**The filter.** One constant-velocity Kalman filter per obstacle, state `[x, y, vx, vy]`, white-acceleration process
noise `q = 0.5 (m/s²)²`:

- lidar update: `H = [I 0]`, `R = 0.05² I` (position only);
- camera update: `H = I`, `R = diag(σp², σp², σv², σv²)` from the message.

Each measurement is applied at its own stamp. The camera is usually later than the lidar; an older measurement
(by `D` seconds) is treated as a measurement of the track's past state, `H B(D)` with `B = [[I, −D I], [0, I]]`, and the
process noise of those `D` seconds is added to `R`. Association: Mahalanobis distance on position, gate χ²(2) = 9.21,
nearest first, one to one. New tracks start from lidar clusters only (a newborn track has a velocity sigma of 1 m/s);
camera tracks only update existing ones. A track is published after 3 updates and dropped after 1.0 s without one.

`fusion_node` publishes `source: fused` on `/obstacles/tracked` at 10 Hz, each track predicted to the message time,
with `position_sigma` and `velocity_sigma` = the square roots of the largest eigenvalues of the position and velocity
blocks of the covariance, and `confidence = 1 / (1 + velocity_sigma)`.

## 2. Risk cost in the hazard layer

Without sigmas (both 0, as the oracle and the plain flow source publish) the layer paints the S7 disks: centres
`p + v (age + t_k)`, `t_k = k · 0.25 s` up to 2 s, radius `R` = obstacle radius + 0.25 m robot + 0.05 m margin, cost
252 falling to 60 over the horizon.

With a sigma the disk at `t_k` is uncertain:

- `σ_k = sqrt(σp² + σv² t_k²)`
- a cell at distance `d` from the centre has occupancy probability `P = Φ((R − d) / σ_k)` (Φ the normal CDF)
- its cost is `round(252 · P · w_k)`, `w_k` the S7 time weight normalised to 1 at `k = 0` (1 → 60/252)
- cells to `R + 3 σ_k` are painted, cells below cost 1 are skipped, nothing reaches 253.

A larger sigma therefore gives a lower peak and a wider footprint. The cell takes the highest cost of the disks that
cover it, as before.

## 3. Time-to-collision speed governor

`ttc_governor` (package `rapid_nav_navigation`) takes the robot's pose from TF (`map ← base_footprint`) and its velocity
from `/odom`. For each track (moved to now by the message age) it computes, with constant velocities, the time of
closest approach `t*` and the distance then `d*`, less both radii and `2 σ(t*)` with `σ(t)² = σp² + σv² t²`. When
`d* < 0` and `0 < t* < 3 s` the track asks for `clamp(t* / 3, 0.2, 1.0)` of the speed; otherwise 100 %. The smallest
request wins; drops apply at once, rises are limited to 20 % per second. The result goes out at 10 Hz as
`nav2_msgs/SpeedLimit` (percentage) on `speed_limit`, which Nav2's controller server applies to its controller.
The node logs the simulated seconds spent under a limit (`limited_s`) and the lowest limit.

## Parameters

| node | parameter | default | meaning |
|---|---|---|---|
| `flow_tracker` | `uncertainty` | `false` | publish confidence = r and the sigmas (the fused launch sets true) |
| `fusion_node` | `link`, `min_points`, `static_clearance` | `0.15`, `3`, `0.15` | lidar clustering (m, points, m) |
| `fusion_node` | `q`, `lidar_sigma`, `init_velocity_sigma` | `0.5`, `0.05`, `1.0` | filter noise |
| `fusion_node` | `gate`, `confirm`, `max_age` | `9.21`, `3`, `1.0` | association and track life |
| `fusion_node` | `rate`, `max_range` | `10.0`, `8.0` | publish rate (Hz), longest scan range used (m) |
| `ttc_governor` | `horizon_s`, `min_limit`, `rise_per_s` | `3.0`, `0.2`, `0.2` | TTC horizon, floor, rise rate |
| `ttc_governor` | `robot_radius`, `max_age_s`, `odom_topic` | `0.25`, `0.5`, `/odom` | robot, track age limit, odometry |

The hazard layer has no new parameter: it uses the risk cost whenever a track carries a sigma.

## Running it

```bash
ros2 launch rapid_nav_bringup sim.launch.py navigation:=true scenario:=crossing hazards:=fused rtf:=0.25
ros2 run rapid_nav_tools nav_run --scenario crossing --controller dwb --hazards fused
```

In Docker:

```bash
scripts/dev.sh nav-smoke crossing dwb fused --rtf 0.25
scripts/dev.sh nav-smoke crossing dwb fused --rtf 0.25 --no-governor
scripts/dev.sh experiment m3
```

The simulation runs at a quarter of real time for the same reason as with `hazards:=flow` (the flow network).

## The ablation

- `flow` against `off`: the camera source alone, as in M2.
- `fused` against `flow`: what the lidar, the filter and the risk cost add.
- `fused` with `governor:=false` (`--no-governor`) against `fused`: what the reactive speed limit adds.

The matrix `m3` (`src/rapid_nav_tools/config/experiments/m3.yaml`) runs crossing, head_on and crowd with dwb and
`off`, `flow`, `fused`, three repeats each, at RTF 0.25. The governor ablation runs are separate `nav-smoke` calls.
