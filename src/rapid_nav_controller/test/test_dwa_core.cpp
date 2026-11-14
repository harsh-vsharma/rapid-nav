// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).
// Tests of the dynamic window library, with hand-checkable numbers.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "rapid_nav_controller/dwa_core.hpp"

namespace rapid_nav_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kResolution = 0.05;

// A grid built cell by cell: a cell takes the cost of the box that contains its centre.
class GridBuilder
{
public:
  GridBuilder(double origin_x, double origin_y, double width, double height)
  : origin_x_(origin_x),
    origin_y_(origin_y),
    size_x_(static_cast<int>(std::round(width / kResolution))),
    size_y_(static_cast<int>(std::round(height / kResolution))),
    costs_(static_cast<std::size_t>(size_x_) * static_cast<std::size_t>(size_y_), 0)
  {
  }

  void fill_box(double x0, double x1, double y0, double y1, std::uint8_t cost)
  {
    for (int my = 0; my < size_y_; ++my) {
      for (int mx = 0; mx < size_x_; ++mx) {
        const double x = origin_x_ + (mx + 0.5) * kResolution;
        const double y = origin_y_ + (my + 0.5) * kResolution;
        if (x >= x0 && x < x1 && y >= y0 && y < y1) {
          costs_[static_cast<std::size_t>(my) * static_cast<std::size_t>(size_x_) +
            static_cast<std::size_t>(mx)] = cost;
        }
      }
    }
  }

  CostGrid build() const
  {
    return CostGrid(origin_x_, origin_y_, kResolution, size_x_, size_y_, costs_);
  }

private:
  double origin_x_;
  double origin_y_;
  int size_x_;
  int size_y_;
  std::vector<std::uint8_t> costs_;
};

// The points of a straight path from (x0, y0) to (x1, y1), count + 1 of them, evenly spaced.
std::vector<Pose2D> line_path(double x0, double y0, double x1, double y1, int count)
{
  std::vector<Pose2D> path;
  const double heading = std::atan2(y1 - y0, x1 - x0);
  for (int i = 0; i <= count; ++i) {
    const double f = static_cast<double>(i) / static_cast<double>(count);
    path.push_back(Pose2D{x0 + f * (x1 - x0), y0 + f * (y1 - y0), heading});
  }
  return path;
}

TEST(DwaGrid, CostAtIsMinusOneOutsideTheGrid)
{
  GridBuilder builder(0.0, 0.0, 1.0, 1.0);
  builder.fill_box(0.0, 0.5, 0.0, 0.5, 254);
  const CostGrid grid = builder.build();
  EXPECT_EQ(grid.cost_at(0.25, 0.25), 254);
  EXPECT_EQ(grid.cost_at(0.75, 0.75), 0);
  EXPECT_EQ(grid.cost_at(-0.01, 0.5), -1);
  EXPECT_EQ(grid.cost_at(1.0, 0.5), -1);
}

TEST(DwaAngles, NormalizeWrapsToPiIncluded)
{
  EXPECT_NEAR(normalize_angle(3.0 * kPi), kPi, 1e-9);
  EXPECT_NEAR(normalize_angle(-kPi), kPi, 1e-9);
  EXPECT_NEAR(normalize_angle(kPi + 0.1), -kPi + 0.1, 1e-9);
  EXPECT_NEAR(normalize_angle(-0.3), -0.3, 1e-12);
}

// 1. The window at rest with dt 0.05: v in [0, 0.04], w in [-0.125, 0.125], 11 x 21 samples.
TEST(DwaWindow, AtRestItSpansTheAccelerationTimesDtAndHasTheSampleCounts)
{
  DwaParams params;
  params.dt = 0.05;
  const std::vector<Velocity> samples = window_samples(Velocity{0.0, 0.0}, params);
  ASSERT_EQ(samples.size(), 11u * 21u);

  double v_min = 1.0;
  double v_max = -1.0;
  double w_min = 1.0;
  double w_max = -1.0;
  for (const Velocity & sample : samples) {
    v_min = std::min(v_min, sample.v);
    v_max = std::max(v_max, sample.v);
    w_min = std::min(w_min, sample.w);
    w_max = std::max(w_max, sample.w);
  }
  EXPECT_NEAR(v_min, 0.0, 1e-12);
  EXPECT_NEAR(v_max, 0.04, 1e-9);
  EXPECT_NEAR(w_min, -0.125, 1e-9);
  EXPECT_NEAR(w_max, 0.125, 1e-9);
}

// 2. The rollouts: (0.5, 0) for 1.5 s ends at (0.75, 0, 0); (0.5, 1.0) ends on the circle.
TEST(DwaRollout, StraightAndArcEndWhereTheKinematicsSay)
{
  GridBuilder builder(-1.0, -1.0, 4.0, 4.0);
  const CostGrid grid = builder.build();
  const DwaParams params;

  const Trajectory straight = rollout(grid, Pose2D{}, Velocity{0.5, 0.0}, params);
  ASSERT_EQ(straight.poses.size(), 15u);
  EXPECT_NEAR(straight.poses.back().x, 0.75, 1e-9);
  EXPECT_NEAR(straight.poses.back().y, 0.0, 1e-9);
  EXPECT_NEAR(straight.poses.back().theta, 0.0, 1e-9);
  EXPECT_TRUE(straight.valid);

  const Trajectory arc = rollout(grid, Pose2D{}, Velocity{0.5, 1.0}, params);
  EXPECT_NEAR(arc.poses.back().x, 0.5 * std::sin(1.5), 1e-9);
  EXPECT_NEAR(arc.poses.back().y, 0.5 * (1.0 - std::cos(1.5)), 1e-9);
  EXPECT_NEAR(arc.poses.back().theta, 1.5, 1e-9);
}

// 3. A free grid, a straight path ahead, moving at 0.3 m/s: no turn, no slowing down.
TEST(DwaCompute, FreeGridStraightPathKeepsTheCourseAndDoesNotSlowDown)
{
  GridBuilder builder(-1.0, -2.0, 5.0, 4.0);
  const CostGrid grid = builder.build();
  const std::vector<Pose2D> path = line_path(0.0, 0.0, 3.0, 0.0, 60);
  const DwaResult result =
    compute(grid, Pose2D{}, Velocity{0.3, 0.0}, path, path.back(), DwaParams{});
  EXPECT_EQ(result.status, DwaStatus::kOk);
  EXPECT_NEAR(result.cmd.w, 0.0, 1e-9);
  EXPECT_GE(result.cmd.v, 0.3 - 1e-9);
}

// 4. A wall across the path with a gap on the left: the command turns left. The wall is 0.6 m
// ahead, not 1 m: a rollout at 0.45 m/s over 1.5 s reaches 0.7 m, so 1 m lies beyond every one.
TEST(DwaCompute, AWallWithAGapOnTheLeftSteersThroughTheGap)
{
  GridBuilder builder(-1.0, -3.0, 6.0, 6.0);
  builder.fill_box(0.6, 0.7, -3.0, 0.0, 254);
  const CostGrid grid = builder.build();
  const std::vector<Pose2D> path = line_path(0.0, 0.0, 2.0, 0.8, 50);
  const DwaResult result =
    compute(grid, Pose2D{}, Velocity{0.45, 0.0}, path, path.back(), DwaParams{});
  EXPECT_EQ(result.status, DwaStatus::kOk);
  EXPECT_GT(result.cmd.w, 0.0);
  EXPECT_TRUE(result.best.valid);

  bool right_turns_blocked = false;
  bool left_turn_open = false;
  for (const Trajectory & trajectory : result.trajectories) {
    if (trajectory.w < -1e-9 && trajectory.v >= 0.4) {
      EXPECT_FALSE(trajectory.valid);
      right_turns_blocked = true;
    }
    if (trajectory.w > 1e-9 && trajectory.valid) {
      left_turn_open = true;
    }
  }
  EXPECT_TRUE(right_turns_blocked);
  EXPECT_TRUE(left_turn_open);
}

// 5. Over 200 seeded random situations, the chosen trajectory never touches an inscribed cell.
TEST(DwaCompute, EveryChosenTrajectoryAvoidsInscribedCellsOver200RandomSituations)
{
  std::mt19937 rng(20261114);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  const DwaParams params;
  int chosen = 0;
  for (int trial = 0; trial < 200; ++trial) {
    GridBuilder builder(-2.0, -2.0, 4.0, 4.0);
    for (int box = 0; box < 40; ++box) {
      const double x = -2.0 + 4.0 * unit(rng);
      const double y = -2.0 + 4.0 * unit(rng);
      builder.fill_box(x, x + 0.1, y, y + 0.1, 254);
    }
    builder.fill_box(-0.35, 0.35, -0.35, 0.35, 0);
    const CostGrid grid = builder.build();

    const Pose2D pose{0.0, 0.0, (2.0 * unit(rng) - 1.0) * kPi};
    const Velocity current{0.5 * unit(rng), -1.0 + 2.0 * unit(rng)};
    const double heading = pose.theta + 0.5 * (unit(rng) - 0.5);
    const Pose2D goal{1.8 * std::cos(heading), 1.8 * std::sin(heading), heading};
    const std::vector<Pose2D> path = line_path(0.0, 0.0, goal.x, goal.y, 36);
    const DwaResult result = compute(grid, pose, current, path, goal, DwaParams{});
    if (result.status == DwaStatus::kNoValidTrajectory) {
      continue;
    }
    ++chosen;
    ASSERT_TRUE(result.best.valid) << "trial " << trial;
    for (const Pose2D & step : result.best.poses) {
      const int cost = grid.cost_at(step.x, step.y);
      EXPECT_GE(cost, 0) << "trial " << trial;
      EXPECT_LT(cost, params.lethal_threshold) << "trial " << trial;
    }
  }
  EXPECT_GT(chosen, 150);
}

// 6. Every direction blocked: no valid trajectory.
TEST(DwaCompute, AllDirectionsBlockedGivesNoValidTrajectory)
{
  GridBuilder builder(-1.0, -1.0, 2.0, 2.0);
  builder.fill_box(-1.0, 1.0, -1.0, 1.0, 254);
  const CostGrid grid = builder.build();
  const std::vector<Pose2D> path = line_path(0.0, 0.0, 2.0, 0.0, 40);
  const DwaResult result =
    compute(grid, Pose2D{}, Velocity{0.3, 0.0}, path, path.back(), DwaParams{});
  EXPECT_EQ(result.status, DwaStatus::kNoValidTrajectory);
}

// 7. A heading error of 2.0 rad: turn in place, v = 0, towards the carrot (a positive w).
TEST(DwaCompute, LargeHeadingErrorRotatesInPlaceTowardsTheCarrot)
{
  GridBuilder builder(-3.0, -3.0, 6.0, 6.0);
  const CostGrid grid = builder.build();
  const std::vector<Pose2D> path =
    line_path(0.0, 0.0, 3.0 * std::cos(2.0), 3.0 * std::sin(2.0), 60);
  const DwaResult result =
    compute(grid, Pose2D{}, Velocity{0.0, 0.0}, path, path.back(), DwaParams{});
  EXPECT_EQ(result.status, DwaStatus::kRotateToHeading);
  EXPECT_NEAR(result.cmd.v, 0.0, 1e-12);
  EXPECT_GT(result.cmd.w, 0.0);
}

// 8. Within the xy tolerance with a yaw error: turn in place to the goal's yaw.
TEST(DwaCompute, InsideTheGoalToleranceTurnsInPlaceToTheGoalYaw)
{
  GridBuilder builder(-1.0, -1.0, 5.0, 2.0);
  const CostGrid grid = builder.build();
  const std::vector<Pose2D> path = line_path(0.0, 0.0, 3.0, 0.0, 60);
  const Pose2D pose{2.95, 0.0, 0.0};
  const Pose2D goal{3.0, 0.0, 1.0};
  const DwaResult result = compute(grid, pose, Velocity{0.0, 0.0}, path, goal, DwaParams{});
  EXPECT_EQ(result.status, DwaStatus::kRotateToGoal);
  EXPECT_NEAR(result.cmd.v, 0.0, 1e-12);
  EXPECT_GT(result.cmd.w, 0.0);
}

// 9. The carrot is a metre ahead on a straight path, and the goal when the path ends first.
TEST(DwaPath, CarrotIsAMetreAheadOnAStraightPathAndTheGoalNearTheEnd)
{
  const std::vector<Pose2D> path = line_path(0.0, 0.0, 3.0, 0.0, 60);
  EXPECT_EQ(nearest_path_index(path, Pose2D{2.7, 0.01, 0.0}), 54u);

  const std::size_t carrot = carrot_index(path, 0, 1.0);
  EXPECT_EQ(carrot, 20u);
  EXPECT_NEAR(path[carrot].x, 1.0, 1e-9);
  EXPECT_EQ(carrot_index(path, 58, 1.0), 60u);
}

}  // namespace
}  // namespace rapid_nav_controller
