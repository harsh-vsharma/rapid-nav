# How the demo works

One command runs everything:

```bash
ros2 launch rapid_nav_bringup sim.launch.py
```

The robot builds a map with SLAM while it patrols six numbered checkpoints in a crowded arena. It sees moving people
and a cart with its lidar and its camera, predicts where they will be, and plans and drives around those predictions.
This page follows the data from the sensors to the wheels and explains what each element in RViz shows. The details of
each part are in the other documents, linked from each section.

## The pieces at a glance

```
 Gazebo arena ── robot: lidar (360°, 10 Hz) · RGB-D camera (90°, 30 Hz) · IMU (200 Hz) · wheels
      │          crowd: 2 walkers + 1 cart on scripted paths (obstacle_driver; they make way for the robot)
      │
      ├─ wheels + IMU ──► EKF ───────────────────────────► odom → base_footprint
      │
      ├─ lidar ─► scan_filter (drops beams on moving tracks) ─► slam_toolbox ─► /map, map → odom
      │                                                           (saved after every checkpoint)
      ├─ camera ─► flow_node (optical flow + depth) ─► flow_tracker (moving regions, own velocity)
      │                                                    │
      ├─ lidar ───────────────────────────────────────► fusion_node (Kalman filter per obstacle)
      │                                                    │
      │                                     /obstacles/tracked: position, velocity, sigmas
      │                                         │                       │
      │                       hazard layer (costmaps: predicted   ttc_governor (speed limit
      │                       positions over 2 s)                 from time to collision)
      │                                         │                       │
      │   demo_mission ── goal ──► Nav2: global planner (A*) ─► DWA controller ─► /cmd_vel
      │
      └─ RViz: robot · lidar outline · camera bodies (hulls + arrows) · SLAM map · checkpoints · plans · predictions
```

## The simulation

- **Robot** ([robot.md](robot.md)): a differential-drive base with a 2D lidar on a turret, an RGB-D camera
  (640 × 480, 90° horizontal field of view, depth to 10 m) and an IMU.
- **Arena**: walls, four crates and two pillars.
- **Crowd**: the scenario `crowd` in `nav_scenarios.yaml`. Two walkers pace back and forth and a cart drives a loop.
  `obstacle_driver` moves them along their scripts. In the demo they make way for the robot: a walker slows down when
  the robot is ahead of them within 1.8 m and stops at 0.9 m (centre to centre), but keeps walking when the robot is
  beside or behind them. The driver knows where the robot really is, because it plays the people, not the robot. The
  robot's own software never sees those true positions.
- **Noise**: the camera, the lidar and the IMU have noise. The wheels have none, so wheel odometry is nearly exact.
  What drift there is comes from the IMU (gyro noise and bias), and SLAM corrects it.

## Where am I: odometry and SLAM

Details: [slam.md](slam.md).

- **EKF** (`robot_localization`): combines the wheel speeds with the IMU's yaw rate and acceleration. It publishes
  `odom → base_footprint`, which is smooth but drifts slowly.
- **SLAM** (`slam_toolbox`, online, asynchronous): matches the lidar scans into a map and publishes `/map` and
  `map → odom`, which corrects the drift. The map frame starts where the robot spawns, so it coincides with the arena
  frame. This is why the checkpoints can be written as fixed arena coordinates.
- **Movers stay out of the map**: `scan_filter_node` blanks every lidar beam that ends inside a tracked moving
  obstacle and feeds the rest to SLAM as `/scan_static`. Without this, people would be drawn into the map as walls.
  The costmaps and the fusion still use the full scan.
- **The map is kept between runs**: after every checkpoint it reaches, `demo_mission` asks slam_toolbox to save its
  pose graph to `~/.ros/rapid_nav/demo_map.posegraph` (and `.data`). The next demo loads it and keeps mapping from it.
  Delete both files to start a fresh map. They grow over time (about 12 MB after two laps).

## What moves around me: perception and fusion

Details: [perception.md](perception.md) and [hybrid.md](hybrid.md).

1. **Optical flow** (`flow_node`): computes the dense pixel motion between consecutive camera frames. The demo uses
   Farnebäck, which needs no network and is cheap on a CPU. The experiments default to RAFT. Each result is a
   `FlowFrame` that carries the flow, the depth at both times and the camera intrinsics.
2. **Camera tracks** (`flow_tracker`): turns every pixel into a 3D point at both times and moves both into the map
   frame with the camera pose at each time. A static point then has zero velocity, whatever the robot does; points that
   still move belong to moving things. These points are grouped into obstacles with a position, a velocity and their
   uncertainty.
3. **Fusion** (`fusion_node`): runs one constant-velocity Kalman filter per obstacle.
   - The **lidar** sees all around the robot but gives positions only: clusters of scan points that are not part of
     the map. The filter estimates velocity from how those positions change from scan to scan.
   - The **camera** gives position and velocity directly, but only in front of the robot.

   So something beside or behind the robot gets its velocity from the lidar alone, and something in front from both.
   The fused tracks go out on `/obstacles/tracked`. No part of the robot's software uses the simulator's true
   positions.

## How do I get there: planning and control

Details: [navigation.md](navigation.md).

- **Hazard layer** (`rapid_nav_hazard`): a costmap layer that places each tracked obstacle at its predicted positions
  for the next 2 s, in 0.25 s steps. The cost is highest now and falls with prediction time. Because the global and the
  local costmaps both have it, the path bends around where a person is going, not only where they are.
- **Global planner**: our A* (`rapid_nav_planner::GridPlanner`) on the costmap, replanned once a second by the
  behaviour tree.
- **Controller**: our dynamic window approach (`rapid_nav_controller::DwaController`). It samples velocities, scores
  the short trajectories they give against the costmap and the path, and drives the best one.
- **Speed governor** (`ttc_governor`): for every track it computes the time and distance of the closest approach, at
  constant velocities. If a collision is predicted within 3 s, it lowers the speed limit, to as little as 20 %. The
  limit drops at once and rises by at most 20 % per second.
- **Recovery**: when the controller finds no free trajectory, Nav2's behaviour tree waits, backs up or spins, then
  retries. The logs then say "no collision-free trajectory" or "failed to make progress".

## The task: `demo_mission`

The robot patrols six checkpoints in a loop: east end (3.5, 0), north-east corner (3.5, 2.8), north (−1, 3.2), west
past the cart's loop (−4, 1), south (−1, −3) and home (0, 0). These spots are clear of the crates, the pillars and
the cart.

- Each checkpoint is one `NavigateToPose` goal in the map frame.
- A failed checkpoint is retried once, then skipped. One that takes longer than 90 s is cancelled and counts as a
  failure. A goal Nav2 rejects while it is still starting up is sent again without counting.
- The first goal also starts the crowd (`/scenario/goal`).
- A goal clicked in RViz ("Nav2 Goal") takes over; the mission then retries its own checkpoint.

## What RViz shows

Every element comes from a topic; the configs are `rapid_nav_bringup/rviz/sim.rviz` and
`rapid_nav_navigation/rviz/navigation.rviz`.

| Element | Topic | Meaning |
|---|---|---|
| Robot | `/robot_description` | The robot model; the only pre-made mesh shown. |
| Faint grey map | `/map` | The SLAM map so far (saved and reloaded across runs). |
| White lines | `/viz/scan_lines` | The lidar outline. Neighbouring beams closer than 0.2 m are joined. |
| Coloured hulls | `/viz/flow` | Each body the camera sees, as a solid convex hull (see below). |
| Arrow and arc on a hull | `/viz/flow` | That body's mean velocity (arrow length = 1 s of travel) and its rotation rate (arc). |
| Amber lines and dots | `/viz/predictions` | Each moving track's position in 2 s: what the planner avoids. |
| Numbered rings | `/viz/mission` | Checkpoints. Amber: the current target; teal: reached this lap; grey: still to come; red: skipped. |
| Blue / green line | `/plan`, `/local_plan` | The global path and the controller's short-range path. |

### Camera bodies (`scene_viz`)

`scene_viz` (in `rapid_nav_perception`) turns each flow frame into bodies. The pure geometry is in `scene_vis.py`. The
steps, in order:

1. **Samples**: one sample per 4 × 4 pixel tile, with the robot's own motion removed as in `flow_tracker`.
2. **Unreliable samples are dropped** in three cases:
   - next to a depth edge, where flow mixes the foreground and the background;
   - where the depth jumps between the two frames, as when the background is uncovered: there the flow is about zero
     but the depth switches between object and background;
   - faster than 3 m/s, which no body here moves.
3. **Noise floor**: a sample counts as moving only above a floor, which is the largest of:
   - a fixed 0.15 m/s;
   - the speed that flow error fakes at the sample's depth;
   - a share of the robot's own apparent motion, since pose and timing errors scale with it;
   - the error a colour/depth misalignment gives on slanted surfaces.

   Samples where the picture has no texture also count as static, because their flow is a guess.
4. **Grouping by velocity gradient**: neighbouring samples join when their velocities differ by less than 0.25 m/s and
   they are less than 0.25 m apart. Floor samples are left out, so a crate and the floor under it stay separate.
5. **Bodies**: pieces that are close and move alike merge into one body. A body is shown as moving only if the
   previous frame had a consistent mover, so flicker from noise stays static.
6. **Drawing**: each body is drawn as its convex hull in one solid colour, which it keeps from frame to frame by
   matching the nearest body of the last frame. Bodies wider than 2.5 m, such as walls, get no hull; the lidar outline
   and the map show those.

Nothing is remembered once it leaves the camera's view. The node takes about 20 ms per frame on a CPU.

## Demo versus experiments

`sim.launch.py` has two modes, chosen by whether you pass `stack`:

- **Demo** (no `stack` argument): navigation with SLAM, fused hazards, the scan filter, the crowd that makes way for
  the robot, Farnebäck flow, the mission and the saved map.
- **Experiments** (`stack:=sim|perception|planning|navigation`): the old defaults (hazards off, no scenario, RAFT flow,
  scripted walkers that do not yield, no mission). `scripts/dev.sh`, `scripts/run_matrix.sh` and `nav_run` set every
  argument of the condition they run ([experiments.md](experiments.md)). The `(experiments)` arguments appear in
  `ros2 launch rapid_nav_bringup sim.launch.py --show-args`.

## Where the code is

| Package | What is in it |
|---|---|
| `rapid_nav_description` | URDF/xacro of the robot and its sensors. |
| `rapid_nav_gazebo` | The arena world and the Gazebo launch and bridge. |
| `rapid_nav_bringup` | `sim.launch.py` (demo and experiments), `scenario.launch.py`, EKF config, RViz config. |
| `rapid_nav_navigation` | Nav2 and SLAM launch and config, behaviour tree, `ttc_governor`, scenarios, RViz config. |
| `rapid_nav_planner` | Our grid planners (A*, Dijkstra, Theta*). |
| `rapid_nav_controller` | Our DWA controller. |
| `rapid_nav_hazard` | The hazard costmap layer (predicted obstacle positions). |
| `rapid_nav_perception` | `flow_node`, `flow_tracker`, `fusion_node`, `scan_filter_node`, `scene_viz`. |
| `rapid_nav_tools` | `demo_mission`, `obstacle_driver`, `nav_run`, the checks and the experiment tools. |
| `rapid_nav_msgs` | `FlowFrame`, `TrackedObstacle`, `TrackedObstacleArray`. |

Tests: `python3 -m pytest` in each package (with the venv's Python, see [setup.md](setup.md)).

## Known limits

- **A hull covers only what the camera sees.** A person close up, or a crate seen from one side, looks smaller or
  flatter than it is.
- **The camera layer is less sure while the robot turns on the spot.** The noise floors hide most of the false motion
  then, and the slower movers can be missed for a moment.
- **The controller sometimes gets boxed in by walkers.** Nav2's recovery then waits or backs up; a checkpoint can take
  a retry.
- **The saved map keeps growing.** Delete `~/.ros/rapid_nav/demo_map.*` from time to time.
- **The walkers yield only in the demo.** The experiments keep them to their script on purpose, because the scenarios
  measure that encounter.
