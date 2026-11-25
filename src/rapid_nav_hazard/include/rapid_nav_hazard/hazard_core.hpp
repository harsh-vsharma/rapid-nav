// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).
//
// Where tracked obstacles will be: predicted disks with a cost that falls with prediction time,
// and their rasterisation into a cost grid. This header has no ROS dependency; the costmap layer
// converts messages and calls it.

#ifndef RAPID_NAV_HAZARD__HAZARD_CORE_HPP_
#define RAPID_NAV_HAZARD__HAZARD_CORE_HPP_

#include <cstdint>
#include <vector>

namespace rapid_nav_hazard
{

struct Obstacle
{
  double x = 0.0;
  double y = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double radius = 0.0;
  double position_sigma = 0.0;  // 1-sigma per axis (m); 0 with velocity_sigma 0: hard disks (S7)
  double velocity_sigma = 0.0;  // 1-sigma per axis (m/s)
};

// The cost of a disk never reaches the inscribed value 253, whatever the parameters say.
constexpr int kMaxHazardCost = 252;

// Fixed for S7 (the spec's table); the layer declares them as ROS parameters with these defaults.
struct HazardParams
{
  double horizon_s = 2.0;
  double time_step_s = 0.25;
  double robot_radius = 0.25;
  double margin = 0.05;
  int max_cost = 252;  // below the inscribed value 253: a prediction is never a wall
  int min_cost = 60;
  double min_speed = 0.05;
  double max_age_s = 0.5;
};

// A predicted position: a disk with a cost. With sigma > 0 the disk is uncertain: a cell at
// distance d from the centre is occupied with probability P = Phi((radius - d) / sigma) and gets
// cost round(peak P) (peak = max_cost w_k, the time weight w_k normalised to 1 at k = 0).
struct Disk
{
  double x = 0.0;
  double y = 0.0;
  double radius = 0.0;
  int cost = 0;
  double sigma = 0.0;
  double peak = 0.0;
};

struct GridSpec
{
  double origin_x = 0.0;  // world position of the lower-left corner of cell (0, 0)
  double origin_y = 0.0;
  double resolution = 0.05;
  int size_x = 0;
  int size_y = 0;
};

struct PaintedCell
{
  int i = 0;
  int j = 0;
  uint8_t cost = 0;
};

struct BoundingBox
{
  bool empty = true;
  double min_x = 0.0;
  double min_y = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
};

// The disks of all obstacles: for k = 0 .. floor(horizon / step), centre p + v (age_s + k step),
// radius + robot_radius + margin, cost round(max - (max - min) k step / horizon). An obstacle
// slower than min_speed gets only k = 0, at p. An obstacle with a sigma gives each disk
// sigma_k = sqrt(position_sigma^2 + velocity_sigma^2 t_k^2), t_k = k step, and peak = the
// unrounded cost.
std::vector<Disk> predict_disks(
  const std::vector<Obstacle> & obstacles, double age_s, const HazardParams & params);

// The world-coordinate box around all disks, uncertain ones to radius + 3 sigma (empty when there
// are none).
BoundingBox bounding_box(const std::vector<Disk> & disks);

// The cells whose centre lies in a disk, each with the maximum cost of the disks covering it.
// Uncertain disks paint the cells to radius + 3 sigma with round(peak P) (cells below 1 skipped,
// at most kMaxHazardCost). Cells outside the grid are dropped.
std::vector<PaintedCell> rasterise(const std::vector<Disk> & disks, const GridSpec & grid);

// The standard normal CDF.
double normal_cdf(double z);

}  // namespace rapid_nav_hazard

#endif  // RAPID_NAV_HAZARD__HAZARD_CORE_HPP_
