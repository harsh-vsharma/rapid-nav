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

The images are processed at a width of 320 pixels by default (the height follows the camera's aspect ratio: 240 for
the 640 × 480 camera); `width:=640` uses them at full size, which `frame_route` expects and which makes RAFT about
4x slower. The camera tracker's settings are tuned at 320. The flow is in pixels at the
processing size, and the camera intrinsics are scaled to the same size.

## Backends

| backend | needs | typical latency, one 320 × 240 pair (measured before the default became 640 × 480; expect about 4x at 640 × 480) | notes |
|---|---|---|---|
| `farneback` | OpenCV (`python3-opencv`), nothing else | 10.6 ms (median; Docker on the Mac, CPU) | always available; the fallback and the backend of CI |
| `raft_small` | PyTorch and torchvision, installed in a virtual environment; weights from torchvision | 221 ms (median; Docker on the Mac, CPU, 12 iterations) | the default |
| `raft_large` | as `raft_small` | 624 ms (median; Docker on the Mac, CPU, 12 iterations) | slower and more accurate |
| `frame_route` | PyTorch; the FlowNetS and SEA-RAFT-S checkpoints (see "The frame_route backend") | 80 ms mean on Sintel at 320 × 240; at 640 × 480, about 160 ms on calm frames and 600–900 ms on fast ones (CPU, benchmark, not in the simulation) | FlowNetS on calm frames, SEA-RAFT-S on fast ones |

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
| `previous_depth` | `sensor_msgs/Image`, `32FC1` | metres, previous frame, same size (the node keeps the last 30 depth images and picks the nearest one to each frame) |
| `camera_info` | `sensor_msgs/CameraInfo` | intrinsics scaled to the processing size |
| `backend` | `string` | the backend that really ran: `farneback`, `raft_small`, `raft_large`, or for `frame_route` the network that ran on this pair, `frame_route:flownets` or `frame_route:searaft` |
| `latency_ms` | `float32` | wall-clock compute time of the flow alone |

The node also publishes `/perception/flow_vis` (`rgb8`): the flow as a colour picture. The hue is the direction of the
flow and the saturation is its size; zero flow is white.

## From flow to moving obstacles

`flow_tracker` turns the flow frames into tracked obstacles for the hazard layer (`hazards:=flow`, topic
`/obstacles/tracked`, `source: flow`). The geometry, in plain words:

1. **Two 3-D points per pixel.** A pixel of the previous image moved to a pixel of the current one (the flow). The depth
   at the first pixel in the previous depth image and at the second pixel in the current depth image give the point in
   the camera frame at both times. Depth and flow are first reduced to the median of each 4 x 4 block (`depth_block`),
   because the depth sensor's noise (7 mm per pixel) over two frames 0.04 s apart would otherwise look like 0.2 m/s.
2. **Both points into the world.** Each point goes through the camera pose in `map` at its own time (TF at the depth
   image's stamp, so a turning robot does not smear the points). A point of the static world then has the same world
   position at both times: its velocity is zero whatever the robot does. That is the removal of the robot's own motion.
3. **Moving points.** A point counts as moving when its horizontal speed is 0.2 to 3.0 m/s and its height above the floor
   is 0.05 to 2.0 m, it lies 0.3 to 4.0 m from the camera and the previous image has texture in two directions around it
   (the smaller eigenvalue of the structure tensor of the block, at least `min_texture` 6.0: flat walls and straight edges
   give flow that is only a guess).
4. **Groups and tracks.** Moving points are binned on a 0.10 m grid; touching cells form a cluster, which needs at least
   10 points and a mean velocity of at least 0.2 m/s (noise has no mean). Clusters are matched to tracks by the nearest
   position within 0.6 m, the velocity is smoothed (0.5), a track is confirmed after two matches and is dropped after
   1.0 s without one. Only confirmed tracks are published, with a radius from the size of the cluster.

| parameter | default | meaning |
|---|---|---|
| `depth_block` | `4` | block size of the median filter on depth and flow (1: off; then `stride` sets the sample grid) |
| `min_depth`, `max_depth` | `0.3`, `4.0` | range of the depth used, in metres |
| `min_point_speed`, `max_point_speed` | `0.20`, `3.0` | speed window of a moving point, m/s |
| `min_height`, `max_height` | `0.05`, `2.0` | height window above the floor, m |
| `min_texture` | `6.0` | smallest structure-tensor eigenvalue per block (0: off) |
| `gate_k`, `flow_sigma_px` | `0.0`, `0.5` | raises the speed floor with depth to `gate_k * z * sigma / (fx * dt)`; off by default |
| `cell`, `min_samples`, `min_speed` | `0.10`, `10`, `0.20` | cluster grid (m), points per cluster, mean speed per cluster (m/s) |
| `gate`, `alpha`, `max_track_age`, `confirm` | `0.6`, `0.5`, `1.0`, `2` | matching distance (m), velocity smoothing, track lifetime (s), matches to confirm |
| `uncertainty` | `false` | also publish the frame's reliability as `confidence` and per-track `position_sigma`, `velocity_sigma` (set by `hazards:=fused`) |

With `hazards:=fused` the camera tracks go to `/obstacles/flow` with their uncertainty, and `fusion_node` fuses them with
lidar clusters in one Kalman filter per obstacle: see [hybrid.md](hybrid.md).

**Why a slower simulation.** RAFT-Small on this CPU needs about 0.6 s for a pair, so at real time it gets two pairs per
simulated second, too few to follow a walker. `rtf:=0.25` (the `rtf` argument of `sim.launch.py`, also `--rtf` of
`scripts/dev.sh`) slows the Gazebo physics to a quarter; the machine then processes about 7.7 pairs per simulated
second for the flow alone and 3 to 4 with the tracker and navigation running. All navigation timings below are in
simulated seconds; `nav_run` records the measured real-time factor.

**Textured movers.** A plain coloured box gives no flow on its own surface, so the scenario obstacles carry a texture
(`rapid_nav_movers`); the walls and the floor are plain.

### How well it tracks

`track_check` (`scripts/dev.sh track-smoke <scenario>`) scores the tracks against the obstacles' true poses, which it
uses for scoring only. An obstacle counts as in view inside the camera's 69 degrees and 0.3 to 6 m; it is detected
when a track lies within 0.5 m. A false track is a confirmed track that has no obstacle within 0.5 m in more than half
of its messages. One run per cell at `rtf:=0.25`, dwb, final parameters:

| scenario | detection (in view) | false tracks per simulated second | median position error (m) |
|---|---|---|---|
| crossing | 0.73 (8 of 11), 0.43 (12 of 28) in the navigation run | 0.41, 0.71 | 0.17 |
| head_on | 0.54 (7 of 13), 0.55 (17 of 31) | 0.29, 0.64 | 0.28 |
| crowd | 0.71 (59 of 83), 0.63 (54 of 86) | 0.58, 0.73 | 0.23 |

The two numbers per cell are two separate runs, which shows the spread. Limits: the false tracks sit mostly on the far
walls and obstacle edges, where the flow error (0.3 to 0.5 px is 0.1 to 0.2 m/s at 4 to 6 m) is as large as the speed
of a slow walker; they are not mostly while turning. The target of at most 0.2 false tracks per second is not met, and
a sensor with less noise or a better flow model is the way to meet it. The gate that raises the speed floor with depth
removes the false tracks but also most walkers (crossing: 1 of 16 in view), so it is off.

## Parameters

The defaults are in `config/perception.yaml`. The launch file exposes four of them as arguments.

| parameter | default | meaning |
|---|---|---|
| `backend` | `raft_small` | `farneback`, `raft_small`, `raft_large` or `frame_route` |
| `device` | `auto` | `auto` (CUDA when PyTorch sees a GPU, else CPU), `cpu` or `cuda` |
| `width` | `320` | processing width in pixels (640: the camera's full size, for `frame_route`); RAFT needs multiples of 8 and the node rounds down |
| `max_rate` | `10.0` | largest processing rate in Hz of wall-clock time (0: no limit) |
| `max_pair_gap` | `0.2` | largest gap in simulated seconds between the two frames of a pair |
| `depth_slop` | `0.04` | largest time difference in seconds between the current RGB frame and the depth image attached to it |
| `weights_path` | `''` | a state dict for RAFT, instead of torchvision's default weights |
| `raft_iterations` | `12` | RAFT refinement iterations |
| `num_threads` | `0` | PyTorch threads (0: the library's default) |
| `publish_vis` | `true` | publish `/perception/flow_vis` |
| `flownet_weights` | `''` | `frame_route`: FlowNetS checkpoint (`''`: `$RAPID_NAV_WEIGHTS/flownets/flownets_EPE1.951.pth.tar`) |
| `searaft_weights` | `Tartan-C-T-TSKH432x960-S` | `frame_route`: SEA-RAFT checkpoint, a file or a checkpoint name (downloaded once) |
| `route_disp_px` | `10.0` | `frame_route`: a pixel moving more than this (px at the processing size) is fast |
| `route_frac` | `0.3` | `frame_route`: a frame with more than this share of fast pixels goes to SEA-RAFT |
| `stream_iters` | `2` | `frame_route`: SEA-RAFT iterations per pair (warm start) |
| `key_iters` | `4` | `frame_route`: SEA-RAFT iterations on the first pair of a stream |

The node prints one startup line, `flow_node: backend=<ran> device=<device> size=<w>x<h> torch=<version or none>`,
and one status line every 5 s with the received, processed and dropped pairs, the median latency and `no_prev_depth`, the number of pairs skipped because no separate depth image was found for the previous frame.

## The frame_route backend

`frame_route` sends whole frames to one of two networks: FlowNetS, which is fast and good on small motion
(sub-pixel to about 1.5 px error on calm and mid-motion scenes) but 10 px or more off on fast motion, and SEA-RAFT-S
(Wang, Lipson and Deng, ECCV 2024), which is accurate on both but several times slower. It is the default method of
our optical-flow study, ported here from that project.

- A frame is fast when more than `route_frac` (0.3) of its pixels, sampled every 4 px, move more than
  `route_disp_px` (10 px at the 640 × 480 processing size, the benchmark's setting). With a smaller `width`, scale
  it down in proportion (5 px at 320 px).
- While the scene is calm, each pair costs one FlowNetS pass and its own flow decides. When that flow shows fast
  motion, the same pair is re-run on SEA-RAFT (the "recheck"), so no fast frame keeps FlowNetS's flow.
- While the scene is fast, each pair costs one SEA-RAFT pass, not both. SEA-RAFT runs as a stream: each frame goes
  through its feature encoder once, and each pair starts from the previous flow (warm start, 2 iterations). Its own
  flow decides when the next pair goes back to FlowNetS. The first pair of a stream gets 4 iterations.

On 54 MPI-Sintel pairs at 640 × 480 on a CPU, it gave an end-point error (EPE) of 2.03 px at 352 ms per pair on
average (median 164 ms, 95th percentile 919 ms). SEA-RAFT-S on every frame gave 1.61 px at 620 ms, and FlowNetS
alone gave 6.24 px at 156 ms. The time per pair is bimodal: plan for the slow frames if something downstream needs
a fixed rate.

Weights: the SEA-RAFT checkpoint is downloaded from HuggingFace (`huggingface.co/MemorySlices/<name>`) on first use,
checked against its SHA-256 and kept in `$RAPID_NAV_WEIGHTS/searaft/` (default `~/.cache/rapid_nav`). The default
`Tartan-C-T-TSKH432x960-S` was fine-tuned on Things, Sintel, KITTI and HD1K; `Tartan-C-T432x960-S` never saw Sintel.
The FlowNetS checkpoint (`flownets_EPE1.951.pth.tar`, 155 MB, too large for the repository) comes from
[ClementPinard/FlowNetPytorch](https://github.com/ClementPinard/FlowNetPytorch) (pretrained models, FlowNetS);
put it in `$RAPID_NAV_WEIGHTS/flownets/` or point `flownet_weights` at it. Without it the node stops at startup
with a message that says where it looked.

```bash
mkdir -p ~/.cache/rapid_nav/flownets
cp flownets_EPE1.951.pth.tar ~/.cache/rapid_nav/flownets/
ros2 launch rapid_nav_bringup sim.launch.py perception:=true flow_backend:=frame_route
```

The network code is vendored without changes to the layer names, so the released checkpoints load strictly:
`rapid_nav_perception/nets/flownets.py` (FlowNetPytorch, MIT) and `rapid_nav_perception/nets/searaft/`
(SEA-RAFT, BSD-3-Clause, licence in that folder). `searaft.py` is the inference loop and `frame_route.py` the router.

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
`flow_node: backend=raft_small device=cpu size=640x480 torch=2.x.y+cpu` (or `device=cuda` on a GPU). If the line reads
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
