# Simulation guide for the team

A short reference for running the key features. Every command below is on `main`. The full details are in
[setup.md](setup.md), [navigation.md](navigation.md), [hybrid.md](hybrid.md) and [experiments.md](experiments.md).

## 1. Set up once

Ubuntu 24.04 + ROS 2 Jazzy (primary) or Ubuntu 26.04 + ROS 2 Lyrical (supported):

```bash
cd ~/ros_ws/rapid-nav
git pull
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

Run `source install/setup.bash` in every new terminal.

## 2. Simulation and teleop

```bash
ros2 launch rapid_nav_bringup sim.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # second terminal
ros2 run rapid_nav_tools sim_check                      # automatic check; the robot moves by itself
```

## 3. Navigation (planners and controllers)

```bash
ros2 launch rapid_nav_bringup sim.launch.py navigation:=true controller:=dwa   # or dwb, rpp
```

Click a goal with **2D Goal Pose** in RViz. `planning:=true` instead shows all seven planners' paths for each goal.

## 4. Moving obstacles and scored runs

Start a scenario (`crossing`, `head_on` or `crowd`), then score a goal run on ground truth in a second terminal:

```bash
ros2 launch rapid_nav_bringup sim.launch.py navigation:=true scenario:=crossing
ros2 run rapid_nav_tools nav_run --scenario crossing --controller dwa
```

`nav_run` prints success, collisions, minimum clearance and time, and saves a result bundle.

## 5. Hazard sources

Give the same value to the launch (`hazards:=`) and to `nav_run` (`--hazards`):

| Value | What feeds the costmap |
|---|---|
| `off` | nothing (plain Nav2) |
| `oracle_static` | true obstacle positions only |
| `oracle` | true positions and velocities (upper bound) |
| `flow` | the robot's camera: optical flow, ego-motion removed, tracked obstacles |
| `fused` | the hybrid: lidar + camera Kalman fusion, uncertainty-aware risk cost, time-to-collision speed governor |

`flow` and `fused` need the slower simulation (`rtf:=0.25`) so the flow network keeps up:

```bash
ros2 launch rapid_nav_bringup sim.launch.py navigation:=true scenario:=crossing hazards:=fused rtf:=0.25
ros2 run rapid_nav_tools nav_run --scenario crossing --controller dwb --hazards fused
```

Add `governor:=false` to the launch to run `fused` without the speed governor.

## 6. Optical flow only

```bash
ros2 launch rapid_nav_bringup sim.launch.py perception:=true flow_backend:=raft_small
```

Backends: `farneback` (no GPU or PyTorch needed), `raft_small`, `raft_large`, `frame_route` (FlowNetS / SEA-RAFT-S).

## 7. Without a native ROS install (Docker)

On a Mac or any machine with Docker, `scripts/dev.sh` runs the same things headless. Prefix with `ROS_DISTRO=lyrical`
for Lyrical.

```bash
scripts/dev.sh image && scripts/dev.sh build && scripts/dev.sh test
scripts/dev.sh smoke                                       # sim_check
scripts/dev.sh nav-smoke crossing dwb oracle               # one scored run
scripts/dev.sh nav-smoke crossing dwb fused --rtf 0.25     # the hybrid (add --no-governor to drop the governor)
scripts/dev.sh experiment pilot                            # a short experiment matrix; also m1, m2, m3
```

Without internet access to PyPI, build the image with `INSTALL_TORCH=0 scripts/dev.sh image`; the flow node then
uses Farnebäck.

## Tips

- Keep `rtf:=0.25` for any camera-based run on a CPU; on a GPU machine try `rtf:=1.0`.
- Results go into a timestamped folder printed at the end of each run; share that folder when reporting a problem.
- Pull before testing, and report which ROS distro you ran on.
