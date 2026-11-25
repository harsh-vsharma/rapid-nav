# Report: changes on `feature/frame-route-backend`

This report compares the branch `feature/frame-route-backend` with `main` at `cae9b48`. It covers 22 changed files
(1236 lines added, 25 removed) over 12 commits, and CI passes on the branch.

## Summary

The branch makes two changes:

1. **A new optical-flow backend, `frame_route`.** It sends each frame to one of two networks: FlowNetS when the
   scene is calm, SEA-RAFT-S when it moves fast. It is the default method of our optical-flow study (the separate
   Robot Vision course project), ported into RAPID-NAV. It implements the README's "fast flow by default, accurate
   flow when a reliability check fails" (stage S8, adaptive flow). It is **opt-in**: the default backend stays
   `raft_small`.
2. **The flow node now processes at 640x480 instead of 320x240.** The camera was already 640x480. Before this
   change, the node shrank the images to half size. This change affects **every** backend, not only
   `frame_route`. See "What to check" before relying on it.

How to use the new backend:

```bash
ros2 launch rapid_nav_bringup sim.launch.py stack:=perception flow_backend:=frame_route
```

It needs PyTorch and two checkpoints; see "Weights".

## 1. The `frame_route` backend

### How it routes frames

1. **Fast frame.** A frame counts as fast when more than `route_frac` (0.3) of its pixels, sampled every 4 px,
   move more than `route_disp_px` (10 px).
2. **Calm scene.** Each pair costs one FlowNetS pass, and FlowNetS's own flow decides the route. If that flow shows
   fast motion (motion onset), the **same pair is re-run on SEA-RAFT**, so no fast frame keeps FlowNetS's flow. This
   re-run is the "recheck" gate.
3. **Fast scene.** Each pair costs one SEA-RAFT pass, and FlowNetS is skipped. SEA-RAFT runs as a stream:
   - each frame goes through the feature encoder once, and its features are reused for the next pair;
   - each pair starts from the previous flow, pushed one frame forward (warm start);
   - each pair gets 2 refinement iterations, and the first pair of a stream gets 4.

   SEA-RAFT's own flow decides when the scene is calm again and the next pair goes back to FlowNetS.
4. **First pair.** The first pair always goes to SEA-RAFT, because there is no history yet.

On every `FlowFrame` message, the `backend` field names the network that ran on that pair: `frame_route:flownets`
or `frame_route:searaft`.

### Expected accuracy and speed

These numbers come from our study's benchmark: 54 MPI-Sintel clean pairs at 640x480, an 8-thread CPU, and the
SEA-RAFT checkpoint that never saw Sintel. They were **not** measured inside the RAPID-NAV simulation.

| Method | EPE (px) | ms per pair, mean | median | p95 |
|---|---|---|---|---|
| FlowNetS alone | 6.24 | 156 | – | – |
| **`frame_route`** | **2.03** | **352** | 164 | 919 |
| SEA-RAFT-S on every frame (warm start, 2 it) | 1.61 | 620 | – | – |

EPE is the mean end-point error, the distance between the estimated and the true flow, in pixels.

- **Trade-off:** `frame_route` runs 1.75x faster than SEA-RAFT on every frame, with about 3x lower error than
  FlowNetS.
- **Time per pair is bimodal:** about 160 ms on calm frames and 600–900 ms on fast ones. The slowest pairs are the
  motion-onset re-runs, where FlowNetS and a SEA-RAFT keyframe both run. Plan for the p95 if something downstream
  needs a fixed rate.
- **On this branch's code at 640x480** (CPU, Sintel frames), it took about 220 ms per FlowNetS pair, 560 ms per
  warm SEA-RAFT pair, and 1 s on the first pair of a stream.
- **At 320x240,** for anyone who sets `width:=320`, use `route_disp_px:=5.0`. In that benchmark, 5, 7.5 and 10 px
  routed the same frames: EPE 1.36 at 80 ms mean, against 0.99 at 113 ms for SEA-RAFT on every frame. 3 px also
  sends medium motion to SEA-RAFT: EPE 1.19 at 88 ms mean, but the median doubles.

### New files

| File | What it is |
|---|---|
| `src/rapid_nav_perception/rapid_nav_perception/frame_route.py` | The router and the backend. `fast_share()` is the fast-pixel test. `FrameRouter` is the recheck gate in plain numpy, with no PyTorch, so CI can test it. `FlowNetSRunner` wraps FlowNetS: input normalisation, padding to a multiple of 64, output scaling. `FrameRouteFlow` is the backend the node uses: it loads both networks and reports `last_ran`. |
| `src/rapid_nav_perception/rapid_nav_perception/searaft.py` | SEA-RAFT inference: checkpoint names with SHA-256 sums, a download-once cache, a `.safetensors` reader that needs no extra package, the staged inference loop, and `StreamingSeaRaft` (feature cache, warm start, fp16 on CUDA). |
| `src/rapid_nav_perception/rapid_nav_perception/nets/flownets.py` | The FlowNetS network, from ClementPinard/FlowNetPytorch (MIT), inference only. |
| `src/rapid_nav_perception/rapid_nav_perception/nets/searaft/` (`raft.py`, `extractor.py`, `layer.py`, `update.py`, `__init__.py`) | The SEA-RAFT network modules, from princeton-vl/SEA-RAFT @ `9137517`, inference only. |
| `src/rapid_nav_perception/rapid_nav_perception/nets/searaft/LICENSE` | SEA-RAFT's BSD-3-Clause licence, which the copied code requires. |
| `src/rapid_nav_perception/rapid_nav_perception/nets/__init__.py` | Package marker. |
| `src/rapid_nav_perception/test/test_frame_route.py` | 7 tests; see "Verification". |
| `report.md` | This report. |

The copied network files keep the original layer names, so the published checkpoints load with `strict=True`. Each
copied file says where it came from and what was removed: training-only code, the upstream forward pass (replaced
by `searaft.py`), and an ImageNet download at construction.

### Changed files

| File | Change |
|---|---|
| `rapid_nav_perception/backends.py` | `make_backend()` accepts `'frame_route'` and a `frame_route=` settings dict. Without PyTorch, `frame_route` falls back to Farnebäck with one warning, as RAFT already did. The error for an unknown backend lists `frame_route`. |
| `rapid_nav_perception/flow_node.py` | Six new parameters, used only by `frame_route`: `flownet_weights`, `searaft_weights`, `route_disp_px`, `route_frac`, `stream_iters`, `key_iters`. The `backend` field of `FlowFrame` uses the backend's `last_ran` when it has one; for the other backends it is unchanged. |
| `config/perception.yaml` | The six parameters with their defaults and a comment. `backend` is still `raft_small`. |
| `launch/perception.launch.py`, `rapid_nav_bringup/launch/sim.launch.py` | Argument descriptions mention `frame_route`. No new arguments. |
| `package.xml` | The description mentions the new backend. No new dependencies: PyTorch stays optional, as for RAFT. |
| `docs/perception.md` | The backend table, the `backend` field values, the parameter table, and a new section "The frame_route backend" (method, numbers, weights, licences). |
| `docs/architecture.md` | One line in the backend list. |

### Weights

- **SEA-RAFT-S** (36 MB) is downloaded from HuggingFace on first use, checked against its SHA-256, and cached in
  `$RAPID_NAV_WEIGHTS/searaft/` (default `~/.cache/rapid_nav`). The default checkpoint,
  `Tartan-C-T-TSKH432x960-S`, was fine-tuned on Things, Sintel, KITTI and HD1K, which suits a robot.
  `Tartan-C-T432x960-S`, which never saw Sintel, can be chosen with `searaft_weights`.
- **FlowNetS** (`flownets_EPE1.951.pth.tar`, 155 MB) is too large for GitHub's 100 MB file limit and has no stable
  direct download. Copy it by hand into `$RAPID_NAV_WEIGHTS/flownets/`, or point `flownet_weights` at it. Without
  it, the node stops at startup with a message saying where it looked; it does not quietly fall back to another
  backend.

## 2. Processing resolution: 640x480

| Where | Before | After |
|---|---|---|
| `width` parameter (`flow_node.py`, `config/perception.yaml`, `perception.launch.py` argument) | 320 | 640 |
| `route_disp_px` default (node, config, `FrameRouter`, `FrameRouteFlow`) | 5.0 | 10.0 |
| `rapid_nav_tools/perception_check.py`, expected flow size (an INFO line, not pass/fail) | 320x240 | 640x480 |
| `docs/perception.md`, `docs/sim-checks.md` (processing size, startup line, parameter table) | 320x240 | 640x480 |

The Gazebo camera (`rgbd_camera`) was already 640x480 at 30 Hz, so nothing changes in Gazebo or the bridge. With
`width` at 640, the node passes the images and the camera intrinsics through without resizing.

To go back to the old behaviour: `width:=320` for all backends, plus `route_disp_px:=5.0` for `frame_route`.

### What to check before relying on it

- **Cost.** Every backend does about 4x the work per pair. On a CPU, `raft_small` will be about 4x slower than the
  221 ms in `docs/perception.md`, which was measured at 320x240 (the table says so). The node will process fewer
  pairs per second and drop more of them. The `hazards:=flow` setup was sized for 320x240: the simulation at a
  quarter of real time (`rtf:=0.25`) and 4 threads. It may need a lower `rtf` or a GPU (used automatically when present).
- **The tracker was tuned at 320x240.** `flow_tracker` works on 4x4-pixel blocks (`depth_block`) and has a
  per-block texture threshold (`min_texture` 6.0). At 640x480:
  - a block covers a quarter of the scene area it used to;
  - image gradients per pixel are smaller, which changes what passes the texture threshold;
  - clusters get about 4x more points, so the 10-point minimum is easier to reach.

  Re-check the false-track rate (the S7 flow checks). Setting `depth_block:=8` keeps the old block size in scene
  terms. The tracker's settings were not changed on this branch.

## The pipeline, end to end

This chain was traced from the launch files, the bridge config and the node sources:

1. **Gazebo** renders the robot's RGB-D camera (640x480, 30 Hz).
2. **`ros_gz_bridge`** (`rapid_nav_gazebo/config/bridge.yaml`) brings `/camera/image`, `/camera/depth_image` and
   `/camera/camera_info` from Gazebo into ROS.
3. **`flow_node`** pairs consecutive RGB frames, computes the flow with the chosen backend, attaches the depth of
   both frames, and publishes `FlowFrame` on `/perception/flow_frame`.
4. **`flow_tracker`** back-projects the flow with depth and removes the robot's own motion through TF. It then
   clusters the moving points and publishes `/obstacles/tracked`.
5. **`HazardLayer`**, in the Nav2 local and global costmaps, paints the obstacles' predicted positions as cost.
6. **The controller** (our DWA, or DWB or RPP) plans around that cost and publishes `/cmd_vel`.
7. **`ros_gz_bridge`** sends `/cmd_vel` back to Gazebo, where the diff-drive plugin moves the robot.

The flow itself never goes back into Gazebo. The loop closes through the robot's motion: only `/cmd_vel` returns.
The full loop runs with `stack:=navigation hazards:=flow`. With `stack:=perception` alone, the chain stops after step 3.

## Verification

| Check | Result |
|---|---|
| `ament_flake8` and `ament_pep257` on `rapid_nav_perception` and `rapid_nav_tools`, and flake8 on `sim.launch.py` | clean |
| `rapid_nav_perception` tests, system Python without PyTorch (as in CI) | 47 passed, 2 skipped (the two PyTorch tests) |
| `rapid_nav_perception` tests, with PyTorch and both checkpoints | 49 passed, including an end-to-end test: a (+3, −2) px shift recovered within 0.5 px by SEA-RAFT and by FlowNetS |
| `rapid_nav_tools` tests | 106 passed |
| Same frames through our study's original router and this port | identical flow (max difference 0.0) and the same route on every pair |
| `frame_route` on 640x480 frames | flow (480, 640, 2); routes as expected |
| GitHub Actions CI on the branch: full-workspace `colcon build` and `colcon test`, dependencies installed with rosdep | passed (run 45543362664, on the 640x480 commit) |

Tests added in `test/test_frame_route.py`:

1. `fast_share` counts the sampled pixels above the threshold.
2. The first pair goes to SEA-RAFT, and FlowNetS does not run.
3. Calm frames cost one FlowNetS pass each.
4. Motion onset re-runs the same pair on SEA-RAFT and keeps SEA-RAFT's flow.
5. Fast frames skip FlowNetS until SEA-RAFT's flow is calm, then hand back to FlowNetS.
6. Without PyTorch, `frame_route` falls back to Farnebäck with the warning.
7. End to end with the real networks. It skips when PyTorch or the checkpoints are missing.

## Not verified

- **Not run in the RAPID-NAV simulation.** The machine where this was written lacks
  `ros-jazzy-rosidl-default-generators`, so `rapid_nav_msgs` could not be built there and the node never ran end to
  end in ROS. CI builds and tests the whole workspace but does not run the simulation. The next step is a run with
  `flow_backend:=frame_route` and the S2 perception check (`perception_check`), then `hazards:=flow`.
- **No timing or tuning on the robot.** The robot's own motion, turning in place in particular, moves the whole
  image. Check how often frames go to SEA-RAFT while the robot drives, and tune `route_disp_px` and `route_frac` if
  FlowNetS is rarely used.
- **Velocity noise.** The tracker turns flow error into velocity error, in proportion to depth and frame rate. Check
  its false tracks with `frame_route` and at 640x480 before using `hazards:=flow`.

## Notes for the reviewer

- The default backend is still `raft_small`. To make `frame_route` the default after testing, change `backend` in
  `perception.yaml`, `perception.launch.py` and `sim.launch.py`, and update the README's status table (S8).
- An earlier branch, `feature/frame-routing-flow`, has the same code with an older commit history. The repository's
  rules block deleting it, so the owner may want to delete it by hand. `feature/frame-route-backend` is the one to
  review.
- Commit `03f9227` is titled "rewrite the report" but holds only a comment fix in `perception.yaml`. The rewrite
  itself is the commit after it. The branch rules block rewriting pushed commits, so the title stays.
- Follow-up: fetch the SEA-RAFT checkpoint in `docker/Dockerfile`, as is done for the RAFT weights, so containers
  need no network at run time.
