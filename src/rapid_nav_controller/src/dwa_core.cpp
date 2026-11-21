// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).

#include "rapid_nav_controller/dwa_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace rapid_nav_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kStraightEpsilon = 1e-6;  // |w| below this is a straight line
constexpr double kTieEpsilon = 1e-12;
constexpr double kPathTolerance = 1e-9;
constexpr double kMinTargetDistance = 1e-3;
constexpr double kObstacleCostScale = 252.0;  // the obstacle critic is the largest cost / 252
constexpr double kInfinity = std::numeric_limits<double>::infinity();

// A closed interval of speeds.
struct Interval
{
  double lo;
  double hi;
};

// The forward speeds reachable in one control step from `current`, within the robot's limits.
Interval velocity_window(double current, const DwaParams & params)
{
  const double reach = params.acc_lim_x * params.dt;
  const double hi = std::min(params.max_vel_x, current + reach);
  const double lo = std::min(std::max(params.min_vel_x, current - reach), hi);
  return Interval{lo, hi};
}

// The yaw rates reachable in one control step from `current`, within the robot's limits.
Interval turn_window(double current, const DwaParams & params)
{
  const double reach = params.acc_lim_theta * params.dt;
  const double hi = std::min(params.max_vel_theta, current + reach);
  const double lo = std::min(std::max(-params.max_vel_theta, current - reach), hi);
  return Interval{lo, hi};
}

double clamp_to(double value, const Interval & interval)
{
  return std::min(std::max(value, interval.lo), interval.hi);
}

// `count` values from lo to hi, both edges included; the midpoint when there is only one.
std::vector<double> evenly_spaced(double lo, double hi, int count)
{
  std::vector<double> values;
  if (count <= 1) {
    values.push_back(0.5 * (lo + hi));
    return values;
  }
  values.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    values.push_back(lo + (hi - lo) * static_cast<double>(i) / static_cast<double>(count - 1));
  }
  return values;
}

// The pose after time t of a constant motion (v, w) from the start pose, in closed form.
Pose2D advance(const Pose2D & start, const Velocity & cmd, double t)
{
  Pose2D pose;
  const double heading = start.theta + cmd.w * t;
  if (std::fabs(cmd.w) < kStraightEpsilon) {
    pose.x = start.x + cmd.v * t * std::cos(start.theta);
    pose.y = start.y + cmd.v * t * std::sin(start.theta);
  } else {
    const double radius = cmd.v / cmd.w;
    pose.x = start.x + radius * (std::sin(heading) - std::sin(start.theta));
    pose.y = start.y - radius * (std::cos(heading) - std::cos(start.theta));
  }
  pose.theta = normalize_angle(heading);
  return pose;
}

// The direction of the path at an index: the segment that leaves it, or the one that enters it at
// the end of the path, or the fallback yaw for a one-point path.
double path_direction(const std::vector<Pose2D> & path, std::size_t index, double fallback)
{
  if (index + 1 < path.size()) {
    return std::atan2(path[index + 1].y - path[index].y, path[index + 1].x - path[index].x);
  }
  if (index > 0) {
    return std::atan2(path[index].y - path[index - 1].y, path[index].x - path[index - 1].x);
  }
  return fallback;
}

// The values rescaled to [0, 1]: the smallest maps to 0, the largest to 1, all zero when equal.
std::vector<double> normalised(std::vector<double> values)
{
  if (values.empty()) {
    return values;
  }
  const auto bounds = std::minmax_element(values.begin(), values.end());
  const double lo = *bounds.first;
  const double range = *bounds.second - lo;
  for (double & value : values) {
    value = range > kTieEpsilon ? (value - lo) / range : 0.0;
  }
  return values;
}

}  // namespace

double normalize_angle(double angle)
{
  // Wrapped to (-pi, pi]: pi stays pi, and -pi becomes pi.
  double wrapped = std::fmod(angle + kPi, kTwoPi);
  if (wrapped <= 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped - kPi;
}

CostGrid::CostGrid(double origin_x, double origin_y, double resolution, int size_x, int size_y,
                   std::vector<std::uint8_t> costs)
: origin_x_(origin_x),
  origin_y_(origin_y),
  resolution_(resolution),
  size_x_(size_x),
  size_y_(size_y),
  costs_(std::move(costs))
{
  const bool sized = size_x > 0 && size_y > 0;
  if (!(resolution > 0.0) || !sized ||
    costs_.size() != static_cast<std::size_t>(size_x) * static_cast<std::size_t>(size_y))
  {
    throw std::invalid_argument("CostGrid: the resolution and size must be positive and match");
  }
}

int CostGrid::cost_at(double x, double y) const
{
  const double fx = std::floor((x - origin_x_) / resolution_);
  const double fy = std::floor((y - origin_y_) / resolution_);
  // The comparisons are false for NaN, so a non-finite point is outside as well.
  if (!(fx >= 0.0 && fy >= 0.0 && fx < size_x_ && fy < size_y_)) {
    return -1;
  }
  const auto mx = static_cast<std::size_t>(fx);
  const auto my = static_cast<std::size_t>(fy);
  return costs_[my * static_cast<std::size_t>(size_x_) + mx];
}

std::vector<Velocity> window_samples(const Velocity & current, const DwaParams & params)
{
  const Interval v_range = velocity_window(current.v, params);
  const Interval w_range = turn_window(current.w, params);
  const std::vector<double> vs = evenly_spaced(v_range.lo, v_range.hi, params.vx_samples);
  const std::vector<double> ws = evenly_spaced(w_range.lo, w_range.hi, params.wz_samples);
  std::vector<Velocity> samples;
  samples.reserve(vs.size() * ws.size());
  for (const double v : vs) {
    for (const double w : ws) {
      samples.push_back(Velocity{v, w});
    }
  }
  return samples;
}

Trajectory rollout(const CostGrid & grid, const Pose2D & start, const Velocity & cmd,
                   const DwaParams & params)
{
  Trajectory trajectory;
  trajectory.v = cmd.v;
  trajectory.w = cmd.w;
  trajectory.valid = true;
  const int steps =
    std::max(1, static_cast<int>(std::round(params.sim_time / params.sim_step)));
  trajectory.poses.reserve(static_cast<std::size_t>(steps));
  for (int i = 1; i <= steps; ++i) {
    const Pose2D pose = advance(start, cmd, static_cast<double>(i) * params.sim_step);
    trajectory.poses.push_back(pose);
    const int cost = grid.cost_at(pose.x, pose.y);
    if (cost < 0 || cost >= params.lethal_threshold) {
      trajectory.valid = false;
    } else {
      trajectory.max_cost = std::max(trajectory.max_cost, cost);
    }
  }
  return trajectory;
}

std::size_t nearest_path_index(const std::vector<Pose2D> & path, const Pose2D & pose)
{
  std::size_t nearest = 0;
  double best = kInfinity;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double dx = path[i].x - pose.x;
    const double dy = path[i].y - pose.y;
    const double squared = dx * dx + dy * dy;
    if (squared < best) {
      best = squared;
      nearest = i;
    }
  }
  return nearest;
}

std::size_t carrot_index(const std::vector<Pose2D> & path, std::size_t nearest, double lookahead)
{
  if (path.empty()) {
    return 0;
  }
  const std::size_t start = std::min(nearest, path.size() - 1);
  if (lookahead <= 0.0) {
    return start;
  }
  double travelled = 0.0;
  for (std::size_t i = start + 1; i < path.size(); ++i) {
    travelled += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    if (travelled >= lookahead - kPathTolerance) {
      return i;
    }
  }
  return path.size() - 1;
}

DwaResult compute(const CostGrid & grid, const Pose2D & pose, const Velocity & current,
                  const std::vector<Pose2D> & path, const Pose2D & goal, const DwaParams & params)
{
  DwaResult result;
  if (path.empty()) {
    return result;
  }
  const std::size_t nearest = nearest_path_index(path, pose);
  const std::size_t carrot = carrot_index(path, nearest, params.lookahead);
  const Pose2D & target = path[carrot];
  const Interval turn_range = turn_window(current.w, params);

  // Rotate first: near the goal, turn in place to its yaw; far from it, turn in place towards the
  // carrot when that lies far off to the side. A turn that would collide is not taken, and the
  // window search decides instead.
  const double goal_distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
  if (goal_distance <= params.xy_goal_tolerance) {
    const double yaw_error = normalize_angle(goal.theta - pose.theta);
    if (std::fabs(yaw_error) <= params.yaw_goal_tolerance) {
      result.status = DwaStatus::kOk;
      result.best = rollout(grid, pose, Velocity{0.0, 0.0}, params);
      return result;
    }
    const double w = clamp_to(params.rotate_gain * yaw_error, turn_range);
    Trajectory turn = rollout(grid, pose, Velocity{0.0, w}, params);
    if (turn.valid) {
      result.status = DwaStatus::kRotateToGoal;
      result.cmd = Velocity{0.0, w};
      result.best = std::move(turn);
      return result;
    }
  } else if (std::hypot(target.x - pose.x, target.y - pose.y) > kMinTargetDistance) {
    const double bearing = std::atan2(target.y - pose.y, target.x - pose.x);
    const double heading_error = normalize_angle(bearing - pose.theta);
    if (std::fabs(heading_error) > params.rotate_to_heading_angle) {
      const double w = clamp_to(params.rotate_gain * heading_error, turn_range);
      Trajectory turn = rollout(grid, pose, Velocity{0.0, w}, params);
      if (turn.valid) {
        result.status = DwaStatus::kRotateToHeading;
        result.cmd = Velocity{0.0, w};
        result.best = std::move(turn);
        return result;
      }
    }
  }

  // The window: roll out every sample, and keep the valid ones.
  const std::vector<Velocity> samples = window_samples(current, params);
  result.trajectories.reserve(samples.size());
  std::vector<std::size_t> alive;
  for (const Velocity & cmd : samples) {
    Trajectory candidate = rollout(grid, pose, cmd, params);
    if (candidate.valid) {
      alive.push_back(result.trajectories.size());
    } else {
      candidate.cost = kInfinity;
    }
    result.trajectories.push_back(std::move(candidate));
  }
  if (alive.empty()) {
    result.status = DwaStatus::kNoValidTrajectory;
    return result;
  }

  // The five critics, each over the surviving trajectories (lower is better).
  const double path_heading = path_direction(path, carrot, goal.theta);
  std::vector<double> path_terms;
  std::vector<double> goal_terms;
  std::vector<double> heading_terms;
  std::vector<double> obstacle_terms;
  std::vector<double> speed_terms;
  for (const std::size_t index : alive) {
    const Trajectory & candidate = result.trajectories[index];
    const Pose2D & end = candidate.poses.back();
    double to_path = kInfinity;
    for (const Pose2D & point : path) {
      to_path = std::min(to_path, std::hypot(end.x - point.x, end.y - point.y));
    }
    path_terms.push_back(to_path);
    goal_terms.push_back(std::hypot(end.x - target.x, end.y - target.y));
    heading_terms.push_back(std::fabs(normalize_angle(end.theta - path_heading)));
    obstacle_terms.push_back(static_cast<double>(candidate.max_cost) / kObstacleCostScale);
    speed_terms.push_back(
      params.max_vel_x > 0.0 ? (params.max_vel_x - candidate.v) / params.max_vel_x : 0.0);
  }
  path_terms = normalised(std::move(path_terms));
  goal_terms = normalised(std::move(goal_terms));
  heading_terms = normalised(std::move(heading_terms));
  obstacle_terms = normalised(std::move(obstacle_terms));
  speed_terms = normalised(std::move(speed_terms));

  // The lowest weighted sum wins; a tie goes to the faster trajectory.
  std::size_t best_slot = alive.size();
  double best_total = kInfinity;
  double best_speed = 0.0;
  for (std::size_t slot = 0; slot < alive.size(); ++slot) {
    const double total = params.path_distance_weight * path_terms[slot] +
      params.goal_distance_weight * goal_terms[slot] +
      params.heading_weight * heading_terms[slot] +
      params.obstacle_weight * obstacle_terms[slot] +
      params.speed_weight * speed_terms[slot];
    Trajectory & candidate = result.trajectories[alive[slot]];
    candidate.cost = total;
    const bool first = best_slot == alive.size();
    const bool better = first || total < best_total - kTieEpsilon;
    const bool tie = !first && std::fabs(total - best_total) <= kTieEpsilon;
    if (better || (tie && candidate.v > best_speed)) {
      best_slot = slot;
      best_total = total;
      best_speed = candidate.v;
    }
  }

  const Trajectory & best = result.trajectories[alive[best_slot]];
  result.status = DwaStatus::kOk;
  result.cmd = Velocity{best.v, best.w};
  result.best = best;
  return result;
}

}  // namespace rapid_nav_controller
