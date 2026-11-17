# Architecture

This document describes the design of the system. Parts marked *(planned)* are not built yet; they arrive in the
stage named in the [README](../README.md) status table.

## Pipeline

1. **Sensors (planned).** RGB-D camera at 640×480 and 30 Hz with metric depth; IMU at 200 Hz; 2D lidar at 10 Hz;
   wheel odometry.

2. **Adaptive flow.** FlowNetS computes the optical flow by default. The reliability statistic is the 95th
   percentile of the flow magnitude. When that statistic says FlowNetS is unreliable, RAFT computes the flow instead.
   On Sintel Clean (team study), routing sends 42.5% of frames to RAFT: end-point error (EPE) 0.95 px at 14.5 FPS.
   FlowNetS alone gives 5.15 px at 65 FPS; RAFT alone gives 0.63 px at 7.9 FPS.

3. **Ego-motion.** The expected flow of a static scene is computed from depth, the camera intrinsics and the camera
   motion between frames. The rotation comes from the gyro, which needs no depth. The translation comes from the EKF
   velocity and the depth. The residual flow is `F_res = F* − F_ego`, where `F*` is the selected flow and `F_ego` is
   the expected flow of a static scene. Regions that move on their own have a large residual.

4. **Hazards.** Residual motion, depth, reliability and persistence over several frames give hazard points. Each
   hazard point is projected ahead along its estimated motion and written into the Nav2 local costmap.

5. **Navigation.** Nav2 with DWB plans and follows the path. AMCL localises the robot on a known map using the lidar.

6. **Evaluation.** Configurations A–F come from the proposal:
   - A–C compare flow sources on a static costmap;
   - D adds residual motion and a dynamic costmap;
   - E adds a global planner;
   - F (stretch goal) adds semantics.

   The geometry-only baseline is Nav2's standard obstacle layer. The measures are collision rate, minimum clearance,
   success, time to goal, path length, replanning count, perception latency and RAFT share.

## Frames

The robot's frames are implemented in S1 and match the frame table in [robot.md](robot.md):

```
odom → base_footprint → base_link → { camera_link → camera_optical_frame,
                                      imu_link, lidar_link, turret_link,
                                      left_wheel_link, right_wheel_link,
                                      front_caster_link, rear_caster_link }
```

`odom → base_footprint` comes from one publisher only: the robot_localization EKF by default, or the simulator's
differential-drive transform with `odom_tf:=wheel`. The `map` frame is added with localisation in S5.

## Navigation

The global side is built in S5 (details in [navigation.md](navigation.md)):

- **Map.** Two occupancy maps (the arena and a maze) are generated from the arena's geometry and a fixed maze and
  served by `map_server`.
- **Localisation.** AMCL, on the laser scan, publishes `map → odom`; `odom → base_footprint` is unchanged.
- **Global costmap.** The static map, inflated by the robot radius (0.25 m) out to 0.55 m.
- **Planner server.** Nav2's planner server with seven planners selected by `planner_id`: our four (A*, A* with
  smoothing, Dijkstra and Theta*, one plugin, `rapid_nav_planner/GridPlanner`) and three from Nav2 for comparison.

The local side (S6, details in [navigation.md](navigation.md)):

- **Local costmap.** Rolling, 4 × 4 m in `odom`, from the lidar, with the inflation layer.
- **Controller server.** One controller under the id `FollowPath`, chosen at launch: our dynamic window approach
  (`rapid_nav_controller::DwaController`, the library `dwa_core` has no ROS dependency), Nav2's DWB, or Regulated Pure
  Pursuit. The velocity commands go to `/cmd_vel` as `Twist`, the message the simulation's bridge takes.
- **Behaviour server.** Spin, back up, drive on heading and wait, used by the recovery branch of the tree.
- **BT navigator.** `navigate_rapid.xml` (a copy of Nav2's tree, with our planner as the default) runs each goal:
  replanning once a second, following the path, and recovering.
- **Goal runs.** `nav_run` sends the scenario's goals and scores them on the simulator's ground truth, not on the wheel
  odometry.

The simulation side of the moving obstacles (S4, details in [navigation.md](navigation.md)):

- **Obstacles.** Movable bodies (a person cylinder, a cart box) spawned into the arena by `obstacle_driver`. Gazebo's
  velocity-control system moves each at the commanded velocity, and its odometry-publisher system publishes the true
  pose; `ros_gz_bridge` carries both topics (`/model/<name>/cmd_vel`, `/model/<name>/odometry`).
- **Driver.** `obstacle_driver` follows each obstacle's scripted path, starts it when `nav_run` announces its goal on
  `/scenario/goal` (and the robot is close enough, when the scenario says so), and publishes `/scenario/ready` and the
  RViz markers `/scenario/obstacles`.
- **Ground truth only for scoring.** The obstacles' true poses are read by `nav_run` and RViz, never by the navigation
  stack: the stack sees them as lidar geometry, which is the baseline.

The hazard layer (velocity-aware cells from the flow) arrives in S7.
