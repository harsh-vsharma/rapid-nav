# RAPID-NAV

*Reliability-Aware Perception for Intelligent Dynamic Navigation*

[![ci](https://github.com/harsh-vsharma/rapid-nav/actions/workflows/ci.yml/badge.svg)](https://github.com/harsh-vsharma/rapid-nav/actions/workflows/ci.yml)

A differential-drive mobile robot in Gazebo that sees moving obstacles with its RGB-D camera. Optical flow (RAFT, or
FlowNetS / SEA-RAFT routed per frame) gives image motion; depth and the robot's pose remove the robot's own motion;
the remaining motion becomes tracked obstacles with velocities, which a velocity-aware Nav2 costmap layer turns into
predicted hazards for the planners and controllers.

A project for AR522 Robot Vision and AR511 Autonomous Mobile Robots at CAIR, IIT Mandi.

## Status

Parts marked done are on `main` and covered by CI; the simulation runs on a CPU at a reduced real-time factor
(`rtf:=0.25`) when the camera source is used.

| Part | Status |
|---|---|
| Robot and simulation (Gazebo Harmonic arena, RGB-D camera, lidar, IMU) | done |
| Optical flow: Farnebäck, RAFT-Small/Large, `frame_route` (FlowNetS / SEA-RAFT-S) | done |
| Ego-motion removal and camera-based obstacle tracks (`flow_tracker`) | done |
| Global planning: our Dijkstra / A* / Theta* plugin vs NavFn, Smac 2D | done |
| Local control: our DWA vs DWB and RPP | done |
| Dynamic scenarios (crossing, head-on, crowd) and ground-truth scoring | done |
| Hazard layer: velocity-aware costmap from tracked obstacles (oracle or camera) | done |
| Experiments: repeated matrices, resumable runner, bootstrap statistics | done |
| Hybrid: lidar–camera Kalman fusion, risk cost, time-to-collision speed governor ([docs/hybrid.md](docs/hybrid.md)) | done |

## Pipeline

```mermaid
flowchart LR
    cam[RGB-D camera] --> flow[optical flow<br/>RAFT or frame_route]
    cam --> depth[depth]
    flow --> ego[ego-motion removal<br/>depth + camera pose]
    depth --> ego
    ego --> trk[moving-obstacle tracks<br/>position, velocity]
    trk --> haz[hazard layer<br/>predicted disks]
    haz --> cost[Nav2 costmaps]
    cost --> plan[planner]
    cost --> ctrl[controller: DWA, DWB or RPP]
    ctrl --> cmd[cmd_vel]
```

See [docs/architecture.md](docs/architecture.md) for the design, [docs/perception.md](docs/perception.md) for the
vision side, [docs/navigation.md](docs/navigation.md) for planning and control, and
[docs/experiments.md](docs/experiments.md) for the experiments. The `frame_route` backend's benchmark is in
[docs/frame-route-report.md](docs/frame-route-report.md).

## Repository layout

- `src/`: ROS 2 packages: the robot description, the Gazebo arena and bridge, the launch files, the `sim_check` tool,
  the maps and the navigation launch, our planner (`rapid_nav_planner`) and the planner checks (`planner_check`), our
  controller (`rapid_nav_controller`), and the tools (`rapid_nav_tools`): the checks, the path viewer and `nav_run`.
- `prototypes/`: reference code, not built.
- `docs/`: setup, development and architecture notes.
- `docker/`: optional build-and-test image for machines without a native ROS 2 install.
- `scripts/`: `dev.sh`, which drives the Docker image.

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (`ros-jazzy-ros-gz`): the primary setup
- or Ubuntu 26.04, ROS 2 Lyrical, Gazebo Jetty (`ros-lyrical-ros-gz`): supported

The full steps are in [docs/setup.md](docs/setup.md).

## Quick start

```bash
mkdir -p ~/ros_ws && cd ~/ros_ws
git clone https://github.com/harsh-vsharma/rapid-nav.git
cd rapid-nav
sudo rosdep init   # only once per machine; skip if it says it is already initialised
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

Start the simulation (Gazebo with the arena and the robot, the odometry and RViz):

```bash
ros2 launch rapid_nav_bringup sim.launch.py
```

Drive the robot with the keyboard, in a second terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Run the automatic check, in a third terminal (the robot drives by itself, so keep clear of it):

```bash
ros2 run rapid_nav_tools sim_check
```

Start the simulation with the global planning stack (the map, AMCL localisation and the seven planners, with RViz paths
for each goal you click with "2D Goal Pose"):

```bash
ros2 launch rapid_nav_bringup sim.launch.py planning:=true
```

Start the navigation stack (the planning stack, Nav2's controller, behaviour and BT navigator servers, and our DWA
controller; `controller:=dwb` or `controller:=rpp` chooses the others). Click a goal with "2D Goal Pose" in RViz:

```bash
ros2 launch rapid_nav_bringup sim.launch.py navigation:=true
```

With the stack running, run a goal scenario and score it on the simulator's ground truth, in a second terminal:

```bash
ros2 run rapid_nav_tools nav_run --scenario tour --controller dwa
```

Add moving obstacles (a person who crosses the robot's path, one who comes head-on, a small crowd) with the `scenario:=`
argument (`crossing`, `head_on` or `crowd`), then run the same scenario:

```bash
ros2 launch rapid_nav_bringup sim.launch.py navigation:=true scenario:=crossing
ros2 run rapid_nav_tools nav_run --scenario crossing --controller dwa
```

Add `hazards:=oracle` to the launch (and `--hazards oracle` to `nav_run`) to paint the predicted positions of the obstacles
into the costmaps from their true velocities, or `hazards:=oracle_static` for their current positions only. With `hazards:=flow` the velocities come from the robot's own
camera instead (RAFT flow, the robot's motion removed with depth and TF); the simulation then runs at a quarter of real
time (`rtf:=0.25`, `--rtf 0.25` for `scripts/dev.sh`) so that the flow network keeps up. `hazards:=fused` fuses the lidar
and the camera in one Kalman filter per obstacle, turns the prediction's uncertainty into an occupancy cost and adds a
time-to-collision speed limit; see [docs/hybrid.md](docs/hybrid.md).

The maps, the planners, the controllers and the goal runs are described in [docs/navigation.md](docs/navigation.md), the repeated experiments in [docs/experiments.md](docs/experiments.md); the
steps for the tester are in [docs/sim-checks.md](docs/sim-checks.md) (sections 10 to 15).

Start the optical flow on the camera (Farnebäck always, RAFT when PyTorch is installed; see
[docs/perception.md](docs/perception.md)):

```bash
ros2 launch rapid_nav_bringup sim.launch.py perception:=true flow_backend:=farneback
```

The robot is described in [docs/robot.md](docs/robot.md). The step-by-step check procedure for testers is in
[docs/sim-checks.md](docs/sim-checks.md).

## Team

Rishi Atreya, Kamalesh Kumar B, Jaspinder Kaur, Harsh Vardhan (CAIR, IIT Mandi).

## Licence

MIT, see [LICENSE](LICENSE).
