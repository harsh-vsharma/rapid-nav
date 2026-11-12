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

## Perception (S2)

This section checks the optical flow on the robot's camera. Do steps 1 to 9 above first; the simulation needs the
S1 robot as well. About 15 minutes.

1. **Build** the same workspace as in step 1 (`colcon build --symlink-install`), with the Python environment for
   PyTorch active if you want to test RAFT (see [perception.md](perception.md), "Installing PyTorch natively").
   Keep the build output if it shows errors.
2. **Farnebäck, with the simulation running:**

   ```bash
   ros2 launch rapid_nav_bringup sim.launch.py perception:=true flow_backend:=farneback
   ```

   The terminal prints a line `flow_node: backend=farneback device=cpu size=320x240 torch=...`. In RViz the image
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
   `flow_backend:=raft_small` (and `flow_device:=cuda` if the machine has an NVIDIA GPU). The startup line must say
   `backend=raft_small`. Keep the second folder and its terminal output.

   If PyTorch is not installed, skip this step and say so when you send the files back.
6. **Send back** the `perception_check_*` folders, `rviz-flow.png`, and the terminal output of the launches with
   errors, together with the machine details from step 9 of the S1 section.
