// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).
//
// The dynamic window approach (Fox, Burgard and Thrun, 1997) for a differential-drive robot, with
// our own critics. This header has no ROS dependency: the Nav2 controller plugin converts the
// costmap and the path, calls compute() and publishes the result.

#ifndef RAPID_NAV_CONTROLLER__DWA_CORE_HPP_
#define RAPID_NAV_CONTROLLER__DWA_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rapid_nav_controller
{

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

struct Velocity
{
  double v = 0.0;  // forward speed, m/s
  double w = 0.0;  // yaw rate, rad/s
};

// Every number of the S6 spec (§0.2 and §0.3). The plugin reads them from its parameters.
struct DwaParams
{
  // The robot's limits, with the controller's margins (§0.2).
  double max_vel_x = 0.5;
  double min_vel_x = 0.0;
  double max_vel_theta = 1.5;
  double acc_lim_x = 0.8;
  double acc_lim_theta = 2.5;
  double dt = 0.05;  // 1 / controller_frequency

  // The window and the rollout.
  int vx_samples = 11;
  int wz_samples = 21;
  double sim_time = 1.5;
  double sim_step = 0.1;

  // Path following.
  double lookahead = 1.0;
  double rotate_to_heading_angle = 0.8;
  double rotate_gain = 1.5;  // in-place turns: w = rotate_gain * error, clamped to the window
  double xy_goal_tolerance = 0.15;
  double yaw_goal_tolerance = 0.25;

  // The critics' weights (each critic is normalised to [0, 1] over the surviving trajectories).
  double path_distance_weight = 32.0;
  double goal_distance_weight = 24.0;
  double heading_weight = 10.0;
  double obstacle_weight = 8.0;
  double speed_weight = 4.0;

  // A trajectory is invalid when one of its poses lies on a cell at or above this cost (inscribed
  // or lethal) or off the grid.
  int lethal_threshold = 253;
};

// A square grid of costs in the costmap's convention: cell (mx, my) is stored at
// my * size_x + mx and covers [origin_x + mx * resolution, origin_x + (mx + 1) * resolution) in x,
// and likewise in y.
class CostGrid
{
public:
  CostGrid(double origin_x, double origin_y, double resolution, int size_x, int size_y,
           std::vector<std::uint8_t> costs);

  // The cost of the cell containing (x, y), or -1 when that point lies outside the grid.
  int cost_at(double x, double y) const;

  double origin_x() const { return origin_x_; }
  double origin_y() const { return origin_y_; }
  double resolution() const { return resolution_; }
  int size_x() const { return size_x_; }
  int size_y() const { return size_y_; }

private:
  double origin_x_;
  double origin_y_;
  double resolution_;
  int size_x_;
  int size_y_;
  std::vector<std::uint8_t> costs_;
};

struct Trajectory
{
  double v = 0.0;
  double w = 0.0;
  std::vector<Pose2D> poses;  // the pose after each step; the start pose is not included
  double cost = 0.0;          // the weighted sum of the normalised critics; infinite when invalid
  int max_cost = 0;           // the largest cell cost along the poses (the obstacle critic)
  bool valid = false;
};

enum class DwaStatus { kOk, kRotateToHeading, kRotateToGoal, kNoValidTrajectory };

struct DwaResult
{
  DwaStatus status = DwaStatus::kNoValidTrajectory;
  Velocity cmd;
  Trajectory best;
  std::vector<Trajectory> trajectories;  // every candidate of the window (empty for turns in place)
};

// The angle wrapped to (-pi, pi].
double normalize_angle(double angle);

// The window: the speeds reachable in one control step, sampled evenly with both edges included.
std::vector<Velocity> window_samples(const Velocity & current, const DwaParams & params);

// Hold (cmd.v, cmd.w) from the start pose for sim_time, in steps of sim_step (exact arcs).
Trajectory rollout(const CostGrid & grid, const Pose2D & start, const Velocity & cmd,
                   const DwaParams & params);

// The index of the path point nearest to the pose (the first one on a tie).
std::size_t nearest_path_index(const std::vector<Pose2D> & path, const Pose2D & pose);

// The first index after `nearest` whose arc length from it reaches `lookahead`; the last index when
// the path ends first.
std::size_t carrot_index(const std::vector<Pose2D> & path, std::size_t nearest, double lookahead);

// One control cycle: the rotations first (towards the carrot, and to the goal's yaw near the goal),
// then the window search with the critics. An empty path gives kNoValidTrajectory.
DwaResult compute(const CostGrid & grid, const Pose2D & pose, const Velocity & current,
                  const std::vector<Pose2D> & path, const Pose2D & goal, const DwaParams & params);

}  // namespace rapid_nav_controller

#endif  // RAPID_NAV_CONTROLLER__DWA_CORE_HPP_
