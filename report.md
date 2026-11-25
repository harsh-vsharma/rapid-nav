# Report: the `frame_route` flow backend

Branch: `feature/frame-route-backend`, based on `main` at `cae9b48`. Nineteen files changed, 1021 lines added and 15
removed. No existing default was changed.

## Summary

This branch adds a fourth optical-flow backend to `rapid_nav_perception`: **`frame_route`**. It is the default
method of our optical-flow study (the separate "Robot vision" course project), ported into RAPID-NAV.

It routes **whole frames** between two networks:

- **FlowNetS** on calm frames. It is fast and its error is sub-pixel to about 1.5 px on small and medium motion.
- **SEA-RAFT-S** (Wang, Lipson and Deng, ECCV 2024) on fast frames, where FlowNetS is 10 px or more off.

This is the "fast flow by default, accurate flow when a reliability check fails" idea from the README's pipeline
diagram (stage S8, adaptive flow). Here the reliability check is FlowNetS's own displacement.

It is opt-in: `backend:=frame_route` (or `flow_backend:=frame_route` on `sim.launch.py`). The default stays
`raft_small`, and nothing changes for anyone who doesn't ask for it.

```bash
ros2 launch rapid_nav_bringup sim.launch.py perception:=true flow_backend:=frame_route
```

## How the method works

The gate is called **recheck**. It is the most accurate gate in our study's comparison.

1. **Fast frame.** A frame counts as fast when more than `route_frac` (0.3) of its pixels, sampled every 4 px, move
   more than `route_disp_px` (10 px at the 640x480 processing size, the benchmark's setting).
2. **Calm scene.** Each pair costs one FlowNetS pass, and FlowNetS's own flow decides the route. If that flow shows
   fast motion (motion onset), the **same pair is re-run on SEA-RAFT**, so no fast frame keeps FlowNetS's flow.
3. **Fast scene.** Each pair costs one SEA-RAFT pass, without FlowNetS. SEA-RAFT runs as a stream:
   - each frame goes through the feature encoder once, and its features are reused for the next pair;
   - each pair starts from the previous flow pushed one frame forward (warm start);
   - each pair gets 2 refinement iterations, and the first pair of a stream gets 4.

   SEA-RAFT's own flow decides when the scene is calm again and the next pair goes back to FlowNetS.
4. **First pair.** The first pair always goes to SEA-RAFT, because there is no history yet.

### Benchmark results from the source project

These are 54 MPI-Sintel clean pairs at 640x480, on an 8-thread CPU, using the C+T SEA-RAFT checkpoint, which never
saw Sintel. They were **not** measured inside RAPID-NAV; see "Not verified" below.

| Method | EPE (px) | ms per pair, mean | median | p95 |
|---|---|---|---|---|
| FlowNetS alone | 6.24 | 156 | – | – |
| **frame routing, recheck** | **2.03** | **352** | 164 | 919 |
| SEA-RAFT-S warm, 2 it, every frame | 1.61 | 620 | – | – |

- **Trade-off:** it runs 1.75x faster than the fastest SEA-RAFT setting, with about 3x lower error than FlowNetS.
- **Bimodal time per pair:** about 160 ms on calm frames and 600–900 ms on fast ones. The slowest pairs are the
  motion-onset re-runs, where FlowNetS and a SEA-RAFT keyframe both run.

At native 1024x436 the same method gave EPE 1.88 at 679 ms per pair, against 1.62 at 1048 ms for SEA-RAFT-S on every
frame. A crop-based third tier was tested there and gave no gain, so it was left out.

### At 320x240 (only relevant with `width:=320`)

RAPID-NAV used to process at 320x240; the processing size is now the camera's full 640x480 (see "Processing
resolution" below). This run is kept for anyone who sets `width:=320`: same 54 Sintel pairs, resized to 320x240,
same CPU and checkpoint, all rows from one run.

| Method | EPE (px) | ms mean | ms median | ms p95 | to SEA-RAFT |
|---|---|---|---|---|---|
| FlowNetS alone | 3.34 | 50 | – | – | 0% |
| **frame routing, recheck, 5 px** | **1.36** | **80** | 54 | 167 | 39% |
| frame routing, recheck, 7.5 px / 10 px | 1.36 | 79 | 51 | 166 | 39% |
| frame routing, recheck, 3 px | 1.19 | 88 | 103 | 171 | 54% |
| SEA-RAFT-S warm, 2 it, every frame | 0.99 | 113 | – | – | 100% |

- **At 320 px, use 5 px** (`route_disp_px:=5.0`, the 640x480 threshold halved). At 5, 7.5 and 10 px the router
  sends exactly the same frames to SEA-RAFT on this data, so the result does not hinge on the exact value. The small
  time differences are timing noise.
- **3 px is the accuracy option.** It also sends `temple_2` (medium motion spread over the whole frame) to
  SEA-RAFT. That cuts its error from 1.41 to 0.38 px and the mean EPE from 1.36 to 1.19, for 8 ms more per pair on
  average, but the median rises from 54 to 103 ms. Set `route_disp_px:=3.0` if medium motion matters more than
  rate.
- **The speed gain is smaller at this size.** Routing is 1.4x faster than SEA-RAFT on every frame here, against
  1.75x at 640x480. SEA-RAFT's cost falls faster with image size than FlowNetS's does (its correlation volume
  grows with the square of the pixel count). On a GPU the ratio changes again.
- Sintel's motion is not the robot's. How often the robot's frames count as fast still has to be measured in the
  simulation.

## Changes, file by file

### New files

| File | What it is |
|---|---|
| `rapid_nav_perception/frame_route.py` | The router and the backend. `fast_share()` is the fast-pixel test. `FrameRouter` is the recheck gate in plain numpy, with no PyTorch, so CI can test it. `FlowNetSRunner` wraps FlowNetS (input normalisation, padding to a multiple of 64, output scaling). `FrameRouteFlow` is the `FlowBackend` the node uses: it loads both networks and exposes `last_ran`. |
| `rapid_nav_perception/searaft.py` | SEA-RAFT inference: the checkpoint list with SHA-256 sums, a download-once cache, a `.safetensors` reader that needs no extra package, the staged inference loop, and `StreamingSeaRaft` (feature cache, warm start via forward splatting, fp16 autocast on CUDA). |
| `rapid_nav_perception/nets/flownets.py` | The FlowNetS architecture, from ClementPinard/FlowNetPytorch (MIT), for inference only. The batch-norm variant, the FlowNetC correlation and the training outputs were removed. |
| `rapid_nav_perception/nets/searaft/{raft,extractor,layer,update}.py` | The SEA-RAFT network modules, vendored from princeton-vl/SEA-RAFT @ `9137517`, for inference only. The upstream `forward`, the correlation block and the padding utilities were removed, because `searaft.py` replaces them. The ImageNet download at construction was removed. |
| `rapid_nav_perception/nets/searaft/LICENSE` | SEA-RAFT's BSD-3-Clause licence, which the vendored code requires. |
| `rapid_nav_perception/nets/__init__.py` | Package marker. |
| `test/test_frame_route.py` | Seven tests; see "Verification". |

The vendored modules keep their **layer names unchanged**, so the released checkpoints load with
`strict=True`. Every vendored file says where it came from and what was changed.

### Changed files

| File | Change |
|---|---|
| `rapid_nav_perception/backends.py` | `make_backend()` accepts `'frame_route'` and a new `frame_route=` settings dict. The no-PyTorch fallback now covers it too: it runs Farnebäck and logs one warning, which names `frame_route` as well. The unknown-backend error lists `frame_route`. |
| `rapid_nav_perception/flow_node.py` | Six new parameters, all used only by `frame_route`: `flownet_weights`, `searaft_weights`, `route_disp_px`, `route_frac`, `stream_iters`, `key_iters`. The `FlowFrame.backend` field now uses the backend's `last_ran` when it has one, so it reads `frame_route:flownets` or `frame_route:searaft` per pair. For every other backend the field is the same as before. |
| `config/perception.yaml` | The six parameters with their defaults and a comment. `backend` is still `raft_small`. |
| `launch/perception.launch.py` | Argument descriptions mention `frame_route`. No new arguments. |
| `rapid_nav_bringup/launch/sim.launch.py` | The `flow_backend` description mentions `frame_route`. |
| `package.xml` | The description mentions the new backend. No new dependencies: PyTorch stays optional, as for RAFT. |
| `docs/perception.md` | A backend table row, the `backend` field values, the parameter table rows, and a new section "The frame_route backend" (method, benchmark, weights, licences). |
| `docs/architecture.md` | One bullet in the backend list. |

## Design decisions

- **Opt-in, not the new default.** Switching the team default changes what everyone's simulation runs, and the
  method hasn't been timed or tuned inside RAPID-NAV yet. To make it the default after that, change `backend` in
  `perception.yaml` and the two launch defaults.
- **The router doesn't need PyTorch.** `FrameRouter` takes any two callables that return numpy flows. CI runs
  without PyTorch, so it still tests the routing logic, including the motion-onset re-run and the handback after
  fast motion, using stub flows.
- **Threshold matches the processing size.** The flow node now processes at 640x480, the size of the benchmark,
  so `route_disp_px` defaults to the benchmark's 10 px. With a smaller `width`, scale it in proportion.
- **Weights are not in the repository.**
  - SEA-RAFT-S (36 MB) is downloaded from HuggingFace on first use, checked against its SHA-256, and cached in
    `$RAPID_NAV_WEIGHTS/searaft/` (default `~/.cache/rapid_nav`).
  - The FlowNetS checkpoint is 155 MB, over GitHub's 100 MB file limit, and has no stable direct download (it is on
    Google Drive). It must be put in `$RAPID_NAV_WEIGHTS/flownets/` by hand, or `flownet_weights` pointed at it.
    Without it, the node stops at startup with an error saying where it looked; it does not quietly fall back to
    another backend.
- **Default SEA-RAFT checkpoint: `Tartan-C-T-TSKH432x960-S`.** It was fine-tuned on Things, Sintel, KITTI and HD1K.
  That is the better choice on a robot, and the source project's node uses it too. `Tartan-C-T432x960-S` (never saw
  Sintel) is available through `searaft_weights`.
- **Pairs dropped by the node are handled.** The node keeps only the newest pair and drops the others.
  `StreamingSeaRaft` compares each pair's first frame with the last frame it encoded. On a gap it re-encodes both
  frames and starts without warm start, so a dropped pair never gives a wrong warm start.
- **`backend` field semantics.** The docs say this field is "the backend that really ran". With routing, that
  changes per pair, so the field names the network. `perception_check` only prints this field, so nothing that
  reads it breaks.

## Verification

| Check | Result |
|---|---|
| `ament_flake8` on `rapid_nav_perception`, and on the edited `sim.launch.py` | clean |
| `ament_pep257` on `rapid_nav_perception` | clean |
| Package tests, system Python without PyTorch (as in CI) | 47 passed, 2 skipped (the two PyTorch tests) |
| Package tests, Python with PyTorch and both checkpoints | 49 passed, including the end-to-end test: a (+3, −2) px shift recovered within 0.5 px by SEA-RAFT (first pair) and by FlowNetS (second pair) |
| Equivalence with the source project | Same frames (alley_2 and ambush_2, 4 pairs each, 320x240) through the original router and this port: identical flow (max difference 0.0) and the same route on every pair |
| `colcon build --packages-select rapid_nav_perception` | builds; `nets/` and `nets/searaft/` are installed |
| GitHub Actions CI (full workspace `colcon build` and `colcon test`, rosdep-installed dependencies) | passed (run 45536760911) |

Tests added in `test/test_frame_route.py`:

1. `fast_share` counts the sampled pixels above the threshold.
2. The first pair goes to SEA-RAFT, and FlowNetS does not run.
3. Calm frames cost one FlowNetS pass each.
4. Motion onset re-runs the same pair on SEA-RAFT and keeps SEA-RAFT's flow.
5. Fast frames skip FlowNetS until SEA-RAFT's flow is calm, then hand back to FlowNetS.
6. Without PyTorch, `frame_route` falls back to Farnebäck with the warning.
7. End to end with the real networks; it skips when PyTorch or the checkpoints are missing.

## Not verified

- **Not run in the RAPID-NAV simulation.** The machine where this was written lacks
  `ros-jazzy-rosidl-default-generators`, so `rapid_nav_msgs` (and the `FlowFrame` message the node publishes) could
  not be built there. The node's code path was therefore not run end to end in ROS. CI builds the whole workspace.
  The next step is a run with `flow_backend:=frame_route` and the S2 perception checks (`perception_check`).
- **Not timed inside RAPID-NAV.** The numbers above are from Sintel on the source project's benchmark, not from
  the simulation, where the simulator shares the CPU. Measure it with `perception_check` and the `latency_ms` field.
- **The threshold was checked on Sintel, not on the robot.** The robot's own motion (turning in place in
  particular) makes the whole image move. Check how often frames go to SEA-RAFT while the robot drives, and tune
  `route_disp_px` and `route_frac` if FlowNetS is rarely used.
- **Velocity noise.** The tracker turns flow error into velocity error in proportion to depth and frame rate. Check
  the tracker's false tracks with `frame_route` before using it with `hazards:=flow`.

## Processing resolution: 640x480

A later commit changes the flow node's processing size from 320x240 to the camera's full 640x480. That is the
resolution our flow study benchmarked at, and the deployment target. It affects every backend, not only
`frame_route`.

| Where | Before | After |
|---|---|---|
| `flow_node.py` parameter `width`, `config/perception.yaml`, `perception.launch.py` argument `width` | 320 | 640 |
| `route_disp_px` (node, config, `FrameRouter` and `FrameRouteFlow` defaults) | 5.0 | 10.0 |
| `perception_check` expected flow size (an INFO line, not a pass/fail check) | 320x240 | 640x480 |
| `docs/perception.md`, `docs/sim-checks.md` (processing size, startup line, parameter table) | 320x240 | 640x480 |

The camera itself was already 640x480 at 30 Hz (`camera_width` and `camera_height` in the URDF), so nothing changes in
Gazebo or the bridge. With `width` 640 the node no longer resizes the images, and the intrinsics are passed through
unscaled.

What to check before relying on it:

- **Cost.** Every backend does about 4x the work per pair. The `raft_small` latency in `docs/perception.md` (221 ms
  at 320x240) will be roughly 4x higher on a CPU, so the node processes fewer pairs per second and drops more. The
  `hazards:=flow` setup (`rtf:=0.25`, 4 threads) was sized for 320x240 and may need a lower `rtf` or a GPU
  (`flow_device:=cuda`). The latency table in `docs/perception.md` is still the old 320x240 measurement and says so.
- **The tracker was tuned at 320x240.** `flow_tracker` reduces depth and flow to 4x4-pixel blocks (`depth_block`),
  and its `min_texture` threshold (6.0) is a per-block structure-tensor eigenvalue. At 640x480 a block covers a
  quarter of the scene area, and image gradients per pixel are smaller. Clusters get about 4x more points, so
  `min_samples` (10) is easier to reach. Re-check the false-track rate (the S7 flow checks) and consider
  `depth_block:=8` to keep the old block footprint.
- `width:=320` (launch argument) restores the old behaviour for all backends; add `route_disp_px:=5.0` for
  `frame_route`.

## The pipeline, end to end

Verified from the launch files, the bridge config and the node sources:

1. **Gazebo** renders the robot's RGB-D camera (`rgbd_camera` sensor, 640x480, 30 Hz).
2. **`ros_gz_bridge`** (`rapid_nav_gazebo/config/bridge.yaml`) brings `/camera/image`, `/camera/depth_image` and
   `/camera/camera_info` from Gazebo to ROS (GZ_TO_ROS).
3. **`flow_node`** pairs consecutive RGB frames, computes the flow with the chosen backend, attaches the depth for
   both frames and publishes `rapid_nav_msgs/FlowFrame` on `/perception/flow_frame` (and the picture on
   `/perception/flow_vis`). It runs with `perception:=true` or `hazards:=flow`.
4. **`flow_tracker`** (only with `hazards:=flow`) back-projects the flow with depth, removes the robot's own motion
   through TF (camera pose in `map` at each frame's time), clusters the moving points and publishes
   `TrackedObstacleArray` on `/obstacles/tracked`.
5. **`rapid_nav_hazard::HazardLayer`** in the Nav2 local and global costmaps (`navigation.yaml`) listens on
   `/obstacles/tracked` and paints the obstacles' predicted positions as cost.
6. **The controller** (our DWA by default, or DWB / RPP) plans around that cost and publishes `/cmd_vel`.
7. **`ros_gz_bridge`** sends `/cmd_vel` back to Gazebo (ROS_TO_GZ), where the diff-drive plugin moves the robot.

So the loop closes in Gazebo through the robot's motion, not through the flow itself. The flow and the tracks stay
on the ROS side; only `/cmd_vel` goes back. The full loop runs with `navigation:=true hazards:=flow`. With
`perception:=true` alone, the chain stops at step 3 (flow and picture only).

## Follow-ups

- Fetch the SEA-RAFT checkpoint in `docker/Dockerfile`, as is done for the RAFT weights, so containers need no
  network at run time. FlowNetS has to stay a manual download unless a stable mirror is found.
- Time `frame_route` against `raft_small` in the simulation (S2 check), and with `hazards:=flow`.
- If it holds up, make it the default backend and update the README's status table (S8, adaptive flow).
