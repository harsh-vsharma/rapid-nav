# Optical flow on the robot's camera

The perception package turns the simulated camera stream into optical flow, the apparent motion of image content
between two frames. Later stages use the flow together with the depth image and the robot's own motion to separate
what the robot moves from what moves on its own. The flow itself does not remove the robot's motion; that is the
ego-motion stage.

## What is computed, and why consecutive pairs

Flow is computed between two consecutive RGB frames, so the gap is one camera period (1/30 s at full rate). A pair
is never formed from a frame and a much older one: the displacement between the two images must stay small, however
slow the machine is. Pairs whose gap is more than 0.2 s of simulated time are dropped. Depth is needed only for the
second frame: the depth image nearest in time to it (within 0.04 s) is attached, with its own stamp.

The flow node keeps only the newest pair waiting. A worker thread takes the newest pair, computes the flow and
publishes the result. A slow backend therefore drops frames instead of building up a queue.

The images are resized to a processing width of 320 pixels (the height follows the camera's aspect ratio, 240 for the
640 × 480 camera). The flow is in pixels at that size. The camera intrinsics are scaled to the same size.

## Backends

| backend | needs | typical latency, one 320 × 240 pair | notes |
|---|---|---|---|
| `farneback` | OpenCV (`python3-opencv`), nothing else | 10.6 ms (median; Docker on the Mac, CPU) | always available; the fallback and the backend of CI |
| `raft_small` | PyTorch and torchvision, installed in a virtual environment; weights from torchvision | 221 ms (median; Docker on the Mac, CPU, 12 iterations) | the default |
| `raft_large` | as `raft_small` | 624 ms (median; Docker on the Mac, CPU, 12 iterations) | slower and more accurate |

The latencies are the time of the flow computation alone, measured with random images (T1 probe). In a running
simulation the latency is higher, because the simulator and the flow share the CPU: in the S2 check on the Mac,
the median was 11.7 ms for Farnebäck and 1.40 s for RAFT-Small, with the real-time factor of the simulation at about
0.4. On a native machine with a GPU, `flow_device:=cuda` runs RAFT on the GPU.

PyTorch is optional. Without it, asking for a RAFT backend gives Farnebäck and one warning:

```
PyTorch not found: using farneback. Install it as in docs/perception.md to use raft_small or raft_large.
```

The message's `backend` field always says which backend really ran.

## The FlowFrame message

`rapid_nav_msgs/msg/FlowFrame`, published on `/perception/flow_frame`:

| field | type | meaning |
|---|---|---|
| `header` | `std_msgs/Header` | stamp of the current (second) frame; frame `camera_optical_frame` |
| `previous_stamp` | `builtin_interfaces/Time` | stamp of the previous (first) frame |
| `flow` | `sensor_msgs/Image`, `32FC2` | pixels, previous → current, x right and y down, at the processing size |
| `depth` | `sensor_msgs/Image`, `32FC1` | metres, current frame, resized to the flow size (nearest neighbour) |
| `camera_info` | `sensor_msgs/CameraInfo` | intrinsics scaled to the processing size |
| `backend` | `string` | the backend that really ran: `farneback`, `raft_small` or `raft_large` |
| `latency_ms` | `float32` | wall-clock compute time of the flow alone |

The node also publishes `/perception/flow_vis` (`rgb8`): the flow as a colour picture. The hue is the direction of the
flow and the saturation is its size; zero flow is white.

## Parameters

The defaults are in `config/perception.yaml`. The launch file exposes four of them as arguments.

| parameter | default | meaning |
|---|---|---|
| `backend` | `raft_small` | `farneback`, `raft_small` or `raft_large` |
| `device` | `auto` | `auto` (CUDA when PyTorch sees a GPU, else CPU), `cpu` or `cuda` |
| `width` | `320` | processing width in pixels; RAFT needs multiples of 8 and the node rounds down |
| `max_rate` | `10.0` | largest processing rate in Hz of wall-clock time (0: no limit) |
| `max_pair_gap` | `0.2` | largest gap in simulated seconds between the two frames of a pair |
| `depth_slop` | `0.04` | largest time difference in seconds between the current RGB frame and the depth image attached to it |
| `weights_path` | `''` | a state dict for RAFT, instead of torchvision's default weights |
| `raft_iterations` | `12` | RAFT refinement iterations |
| `num_threads` | `0` | PyTorch threads (0: the library's default) |
| `publish_vis` | `true` | publish `/perception/flow_vis` |

The node prints one startup line, `flow_node: backend=<ran> device=<device> size=<w>x<h> torch=<version or none>`,
and one status line every 5 s with the received, processed and dropped pairs and the median latency.

## Running it

The simulation starts the flow node when asked:

```bash
ros2 launch rapid_nav_bringup sim.launch.py perception:=true
ros2 launch rapid_nav_bringup sim.launch.py perception:=true flow_backend:=farneback
ros2 launch rapid_nav_bringup sim.launch.py perception:=true flow_backend:=raft_small flow_device:=cuda
```

The launch file can also be run alone, while a simulation is already running:

```bash
ros2 launch rapid_nav_perception perception.launch.py backend:=farneback
```

`ros2 run rapid_nav_tools perception_check` runs the checks of the flow pipeline (see
[sim-checks.md](sim-checks.md), "Perception (S2)"). Its checks use the robot's own sensors and need no flow ground
truth: the depth image against the lidar, the flow of a still scene, the flow while driving forward, and the flow
while turning in place against the gyro's prediction.

## Installing PyTorch natively

The repository never requires PyTorch to build or test. It is needed only to run the RAFT backends. It goes into a
virtual environment that still sees the ROS packages and the system numpy, so that nothing ROS-related changes. The
numpy must stay 1.x, because the ROS binaries are built against it.

Source the ROS environment first, then create and activate the environment, then install PyTorch. The
environment must be active when the workspace is built, so that the flow node's script runs with the environment's
Python.

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages ~/venvs/rapid-nav
source ~/venvs/rapid-nav/bin/activate
# CPU only:
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
# with an NVIDIA GPU instead:
# pip install torch torchvision
python3 -c "import torch, torchvision, numpy; print(torch.__version__, torchvision.__version__, numpy.__version__)"
```

The numpy version printed must start with `1.`. If pip replaced numpy, run `pip install "numpy<2"` and run the
check again.

Then build the workspace from the same shell, with the environment still active:

```bash
cd ~/ros_ws/rapid-nav
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix rapid_nav_perception
head -n 1 "$(ros2 pkg prefix rapid_nav_perception)/lib/rapid_nav_perception/flow_node"
```

The last line must print `#!/usr/bin/env python3`, so the flow node runs with the environment's Python.

To confirm that RAFT is used, start the simulation with `perception:=true` and look for the startup line. It should read
`flow_node: backend=raft_small device=cpu size=320x240 torch=2.x.y+cpu` (or `device=cuda` on a GPU). If the line reads
`backend=farneback torch=none`, PyTorch was not found: check that the environment is active in the shell that
runs the launch file.

The tests that need PyTorch run only with the environment's Python:

```bash
cd src/rapid_nav_perception
python3 -m pytest -q test
```

Without PyTorch, these tests are skipped and everything else passes.

## The Docker image

The development image (`docker/Dockerfile`) installs the same environment at `/opt/venv` (CPU wheels) and fetches
the RAFT weights at build time, so runs need no network. See [development.md](development.md).
