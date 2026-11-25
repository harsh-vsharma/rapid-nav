# Checking the simulation

This is the procedure for checking the robot and the simulation on an Ubuntu 24.04 machine with ROS 2 Jazzy and
Gazebo Harmonic installed natively (see [setup.md](setup.md)). It takes about 30 minutes. Send back the zipped folder
listed in step 9.

Run each command from the workspace folder, `~/ros_ws/rapid-nav`, unless a step says otherwise. Use a separate
terminal for each command that keeps running.

## 1. Update and build

```bash
cd ~/ros_ws/rapid-nav
git fetch
git switch feature/robot-sim
git pull
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

The build must finish with no errors. Keep the terminal output if it shows any.

## 2. The model alone

```bash
ros2 launch rapid_nav_description display.launch.py
```

RViz opens with the robot, and a window with joint sliders opens too. Move the wheel sliders and check that the wheels
turn. Save a screenshot as `rviz-model.png`, then close the launch with Ctrl+C.

## 3. The simulation

```bash
ros2 launch rapid_nav_bringup sim.launch.py
```

Gazebo opens with the arena and the robot, and RViz opens too. Wait until the robot has stopped moving (a few seconds).
Save three screenshots:

- `gazebo-robot.png`: a close-up of the robot in Gazebo;
- `gazebo-arena.png`: an overview of the arena in Gazebo;
- `rviz.png`: RViz with the laser scan, the camera image and the depth image panels visible.

Leave this launch running for steps 4 to 7.

## 4. The automatic check

In a second terminal, from the same folder and after `source install/setup.bash`:

```bash
ros2 run rapid_nav_tools sim_check
```

The robot drives about 1.2 m forward and then turns 180 degrees in place. **Do not touch the robot or the keyboard
during this.** The check takes about one minute. It prints a table of checks with PASS, WARN or FAIL, and writes a
folder `sim_check_<time>/` in the current folder. Keep the terminal output.

## 5. Teleop

Leave the simulation running and, in a third terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Click the terminal and drive the robot once around `crate_1`, which stands at x = 2.0 m, y = 1.5 m in the arena
(the robot starts at the origin). Then answer in writing:

- Does the laser scan stay on the walls in RViz?
- Does the robot shake, slide or tip at any point?

## 6. The point cloud

In RViz, tick the display "PointCloud2" on the topic `/camera/points`. Check whether the cloud lines up with the laser
scan and with the walls. Save a screenshot as `rviz-points.png`, and answer in writing: does the cloud line up?

Untick the display again when you are done.

## 7. The transform tree

```bash
ros2 run tf2_tools view_frames
```

The command writes a PDF named `frames_<time>.pdf` to the current folder. Keep it, renamed to `frames.pdf`.

## 8. The wheel odometry mode

Stop the simulation (Ctrl+C in the launch terminal) and start it again in wheel odometry mode:

```bash
ros2 launch rapid_nav_bringup sim.launch.py odom_tf:=wheel
```

In the second terminal, run the check again:

```bash
ros2 run rapid_nav_tools sim_check --odom-tf wheel
```

This writes a second `sim_check_<time>/` folder. Keep the terminal output of both runs.

## 9. Send back

Zip the following into one file and send it to the owner:

- both `sim_check_*` folders (the one from step 4 and the one from step 8);
- `rviz-model.png`, `gazebo-robot.png`, `gazebo-arena.png`, `rviz.png`, `rviz-points.png`;
- `frames.pdf`;
- your written answers to the questions in steps 5 and 6;
- the terminal output of any launch that printed errors (the terminal text, or a copy-paste into a text file).

Also send your machine's details: the output of `nvidia-smi` if you have an NVIDIA GPU, otherwise `lscpu | head -n 20`,
and `free -h`.

## 10. Global planning (S5)

This part checks the map, the localisation and the planners. It needs a native ROS 2 Jazzy install (or the Docker image
for the offline check). Build with `colcon build --symlink-install` as in step 1, and `source install/setup.bash` in
every terminal.

**a. Start the simulation with the planning stack.** The robot starts at the origin of the arena and RViz opens with the
planning layout (map, particle cloud, costmap, scan and one path per planner):

```bash
ros2 launch rapid_nav_bringup sim.launch.py stack:=planning
```

**b. Localisation in RViz.** Wait about 20 seconds. The particle cloud (the small arrows) should collapse onto the robot
and the map's walls should line up with the laser scan. Save a screenshot as `rviz-map.png`.

**c. Goals in RViz.** Use the "2D Goal Pose" tool to click three goals in open floor, one of them behind a crate. After
each click, seven coloured lines appear, one per planner (`/planned_paths/<planner id>`), and the log in the launch
terminal prints each planner's length and planning time. The lines should stay clear of the crates and walls. Save a
screenshot with the paths as `rviz-paths.png`.

**d. The automatic checks.** This drives the robot (the same drive as step 4), so do not send goals while it runs. In a
second terminal:

```bash
ros2 run rapid_nav_tools planner_check --mode sim --map arena
```

It prints the checks and writes a `planner_check_arena_<time>/` folder with `summary.md`, `table_arena.md`,
`paths_arena.png`, `results.csv` and `results.json`. The check takes several minutes. Keep the terminal output.

**e. The offline maze check.** This needs no simulation. Stop the simulation (Ctrl+C), then start the planning stack
without the robot:

```bash
ros2 launch rapid_nav_navigation planning.launch.py localization:=static use_sim_time:=false map:=maze
```

In a second terminal:

```bash
ros2 run rapid_nav_tools planner_check --mode offline --map maze
```

This writes a `planner_check_maze_<time>/` folder. Stop the launch with Ctrl+C when it has finished.

**What to send back** for this part: `rviz-map.png` and `rviz-paths.png`; the two `planner_check_*` folders from steps d and
e; the terminal output of both checks; and any error printed by the launch. The meaning of the checks, and the
numbers they should give, are in [navigation.md](navigation.md).

## 11. Navigation (S6)

This part drives the robot to goals with the navigation stack: the Nav2 controller and behaviour servers, our DWA
controller, and the goal runs scored on the simulator's ground truth. It needs a native ROS 2 Jazzy install (or the
Docker image). Build as in step 1, and `source install/setup.bash` in every terminal. Stop any earlier simulation first.

**a. Start the navigation stack.** The default controller is ours (`dwa`); the other two are `dwb` and `rpp`:

```bash
ros2 launch rapid_nav_bringup sim.launch.py stack:=navigation
```

RViz opens with the navigation layout (the map, the global costmap, scan, the planned paths, and `/plan` in cyan and
`/local_plan` in green). The map starts nearly empty and grows as SLAM sees more of the arena; wait until it shows the
walls around the robot (about 10 seconds). In a
second terminal, `ros2 lifecycle get /bt_navigator` should print `active`.

**b. Three goals in RViz.** Use the "2D Goal Pose" tool to click three goals in open floor, one at a time, and let each
one finish before the next. The robot should drive to each goal without hitting a crate or a wall. While it drives,
`/plan` (the global path) and `/local_plan` (the chosen trajectory) should be visible. Save a screenshot in the middle
of a run, with both paths, as `rviz-nav.png`.

**c. The goal runs.** Keep RViz open, but send no goals while a run goes: the robot drives by itself. In a second
terminal:

```bash
ros2 run rapid_nav_tools nav_run --scenario tour --controller dwa
```

It writes a `nav_run_tour_dwa_<time>/` folder with `summary.md`, `results.json`, `environment.txt` and
`trajectories_tour.png`, and it prints the same table. The run takes several minutes. Then stop the launch (Ctrl+C),
start it again with the other controllers, and run each one the same way:

```bash
ros2 launch rapid_nav_bringup sim.launch.py stack:=navigation controller:=dwb
ros2 run rapid_nav_tools nav_run --scenario tour --controller dwb
ros2 launch rapid_nav_bringup sim.launch.py stack:=navigation controller:=rpp
ros2 run rapid_nav_tools nav_run --scenario tour --controller rpp
```

Last, the `unmapped` scenario (two boxes that are not in the map) with our controller. Restart the launch first, because
the boxes stay in the world:

```bash
ros2 launch rapid_nav_bringup sim.launch.py stack:=navigation controller:=dwa
ros2 run rapid_nav_tools nav_run --scenario unmapped --controller dwa
```

The exit status of `nav_run` is 0 when every check of `dwa` passes (every goal reached by the ground truth, no
collision, a body clearance of at least 0.05 m). The `dwb` and `rpp` rows are information only.

**d. What to watch.** The robot's motion should be smooth, not hesitant or oscillating. Note any goal that Nav2 reports
as failed or canceled although the ground-truth row says success, and any "Failed to make progress" or "Timed out"
message in the launch terminal.

**What to send back** for this part: `rviz-nav.png`; the five bundles (`tour` with the three controllers, `unmapped` with
`dwa`), each with its `summary.md`, `results.json`, `trajectories_*.png` and `environment.txt`, and the terminal output of
each launch; a sentence on the robot's
motion (smooth, hesitant, oscillating) for each controller; and any error printed. The meaning of the checks and the
numbers they should give are in [navigation.md](navigation.md).

## 12. Moving obstacles (S4)

This part adds people and a cart that move along scripted paths, and scores the navigation stack against them. It needs
the same install as step 11. Stop any earlier simulation first, and restart the launch before each run (the obstacles
stay in the world).

**a. See the obstacles.** Start the navigation stack with the `crossing` scenario:

```bash
ros2 launch rapid_nav_bringup sim.launch.py stack:=navigation scenario:=crossing
```

In Gazebo an orange cylinder (the walker) stands at (1.5, -2.0) and does not move yet. In RViz the display "Moving
obstacles" shows it at the same place, and `ros2 topic echo --once /scenario/ready` prints `data: true`. Take a
screenshot of both as `scenario-start.png`.

**b. Run it.** In a second terminal, keeping clear of the robot:

```bash
ros2 run rapid_nav_tools nav_run --scenario crossing --controller dwa
```

When goal 2 is sent, the walker should start walking north once the robot is about 2 m from the crossing point, and it
should stop at y = 0.8. In RViz the marker should follow the walker in Gazebo. The run writes
`nav_run_crossing_dwa_<time>/` with `summary.md`, whose first table says whether the scenario played out as scripted
(every row should be PASS), then the navigation table, `results.json`, `environment.txt` and `trajectories_crossing.png`.
Then repeat with `head_on` and `crowd` (restart the launch with `scenario:=head_on` or `scenario:=crowd` first), and
with `controller:=dwb` and `controller:=rpp` if time allows.

**c. What to watch.** Whether the walker moves smoothly and in a straight line, whether the robot sees it (the local
costmap marks it) and what the robot does: slows, stops, goes around or touches it. A FAIL in the validity table is
worth a sentence on what happened (for example the robot pushed the walker).

**What to send back** for this part: `scenario-start.png`; the bundles of the three scenarios with `dwa`; one sentence
per scenario on how the robot behaved; and any error printed. The measures and the numbers measured on Docker are in
[navigation.md](navigation.md).

## 13. Perception (S2)

This section checks the optical flow on the robot's camera. Do steps 1 to 9 above first; the simulation needs the
S1 robot as well. About 15 minutes.

1. **Build** the same workspace as in step 1 (`colcon build --symlink-install`), with the Python environment for
   PyTorch active if you want to test RAFT (see [perception.md](perception.md), "Installing PyTorch natively").
   Keep the build output if it shows errors.
2. **Farnebäck, with the simulation running:**

   ```bash
   ros2 launch rapid_nav_bringup sim.launch.py stack:=perception flow_backend:=farneback
   ```

   The terminal prints a line `flow_node: backend=farneback device=cpu size=640x480 torch=...`. In RViz the image
   panel "Flow" shows the flow picture. Leave the launch running.
3. **Teleoperate** in a second terminal (`ros2 run teleop_twist_keyboard teleop_twist_keyboard`) for about 30
   seconds: drive forward, then turn on the spot. Take a screenshot of RViz with the flow picture visible, as
   `rviz-flow.png`, while the robot is turning.
4. **The check** in a third terminal, while the simulation runs and the robot is not touched:

   ```bash
   ros2 run rapid_nav_tools perception_check
   ```

   It drives the robot 2.4 m forward and then turns it once around on the spot (about 30 s of simulated time).
   **Do not touch the robot or the keyboard during this.** It prints a table and writes a
   `perception_check_<time>/` folder. Keep the terminal output.
5. **PyTorch, if installed:** stop the simulation (Ctrl+C), then start it again with RAFT and repeat steps 2 to 4 with
   `flow_backend:=raft_small` (an NVIDIA GPU is used automatically when present). The startup line must say
   `backend=raft_small`. Keep the second folder and its terminal output.

   If PyTorch is not installed, skip this step and say so when you send the files back.
6. **Send back** the `perception_check_*` folders, `rviz-flow.png`, and the terminal output of the launches with
   errors, together with the machine details from step 9 of the S1 section.

## 14. Hazard layer (S7)

This section runs the crossing scenario with the hazard layer on, fed by the true obstacle velocities (`oracle`). It
needs the install of step 11. Stop any earlier simulation first.

**a. Start it.**

```bash
ros2 launch rapid_nav_bringup sim.launch.py stack:=navigation scenario:=crossing hazards:=oracle
```

`ros2 topic echo --once /obstacles/tracked` should print the walker with `source: oracle`.

**b. Run it and look.** In a second terminal, keeping clear of the robot:

```bash
ros2 run rapid_nav_tools nav_run --scenario crossing --controller dwa --hazards oracle
```

In RViz show the local costmap (`/local_costmap/costmap`). While the walker walks north, a band of cost should extend
ahead of it, strongest at the walker and fading about 1.5 m ahead, and it should leave nothing behind when the walker
has passed. Take a screenshot of the local costmap with the band visible as `local-costmap-hazard.png`. The run writes
`nav_run_crossing_dwa_oracle_<time>/`; its `summary.md` starts with `hazards: oracle`.

**What to send back:** `local-costmap-hazard.png`, the `nav_run_crossing_dwa_oracle_*` folder, one sentence on whether the
band was ahead of the walker and whether anything stayed behind, and any error printed. The numbers measured on Docker are
in [navigation.md](navigation.md).

## 15. Camera-based obstacle tracks (S3)

This section runs the crossing scenario with the hazard layer fed by the robot's own camera (`hazards:=flow`), and scores
the tracks against the true obstacle poses. It needs the install of step 11 and the RAFT weights of the perception
section. With a CUDA GPU PyTorch uses it and the simulation can run faster: try `--rtf 1.0` and read how many frame pairs
per simulated second the `flow_node` status line reports (the Docker CPU runs get 3 to 4 at `rtf` 0.25, which tracks the
walkers only roughly). Without a GPU keep `--rtf 0.25`; a run then takes a few minutes of wall time. Stop any earlier simulation
first.

**a. Score the tracks.**

```bash
scripts/dev.sh track-smoke crossing --rtf 0.25
```

(or, natively, `ros2 launch rapid_nav_bringup sim.launch.py stack:=navigation scenario:=crossing hazards:=flow rtf:=0.25`
with `ros2 run rapid_nav_tools track_check --scenario crossing --out <folder>` and `nav_run --hazards flow` in two more
terminals.) It writes `track_check_crossing_<time>/` with `summary.md`, `results.json`, a picture of true and tracked
paths and the goal run's own bundle. `ros2 topic echo --once /obstacles/tracked` during the run prints tracks with
`source: flow`.

**b. Navigate with it.**

```bash
scripts/dev.sh nav-smoke crossing dwb flow --rtf 0.25
```

The `summary.md` of the run starts with `hazards: flow` and lists the measured real-time factor; every scenario-validity
row should pass.

**What to send back:** the `track_check_crossing_*` folder (its `summary.md` and `tracks_crossing.png`), the
`nav_run_crossing_dwb_flow_*` folder, whether a CUDA GPU was used and the `flow_node` status line (pairs per second), and
any error printed. The numbers measured on Docker are in [perception.md](perception.md) and
[navigation.md](navigation.md).
