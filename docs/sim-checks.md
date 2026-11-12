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
ros2 launch rapid_nav_bringup sim.launch.py planning:=true
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
