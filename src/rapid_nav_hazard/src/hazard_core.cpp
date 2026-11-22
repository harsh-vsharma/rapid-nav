// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).

#include "rapid_nav_hazard/hazard_core.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace rapid_nav_hazard
{

std::vector<Disk> predict_disks(
  const std::vector<Obstacle> & obstacles, double age_s, const HazardParams & params)
{
  std::vector<Disk> disks;
  const double age = std::max(age_s, 0.0);
  const double step = params.time_step_s > 0.0 ? params.time_step_s : params.horizon_s;
  const int steps = static_cast<int>(std::floor(params.horizon_s / step + 1e-9));
  for (const auto & o : obstacles) {
    const double speed = std::hypot(o.vx, o.vy);
    const bool moving = speed >= params.min_speed;
    const int last = moving ? steps : 0;
    const double disk_radius = o.radius + params.robot_radius + params.margin;
    for (int k = 0; k <= last; ++k) {
      const double t = k * step;
      double cost = params.max_cost;
      if (params.horizon_s > 0.0) {
        cost = params.max_cost - (params.max_cost - params.min_cost) * t / params.horizon_s;
      }
      Disk d;
      d.x = o.x + (moving ? o.vx * (age + t) : 0.0);
      d.y = o.y + (moving ? o.vy * (age + t) : 0.0);
      d.radius = disk_radius;
      d.cost = std::min(params.max_cost, static_cast<int>(std::lround(cost)));
      disks.push_back(d);
    }
  }
  return disks;
}

BoundingBox bounding_box(const std::vector<Disk> & disks)
{
  BoundingBox box;
  for (const auto & d : disks) {
    if (box.empty) {
      box.empty = false;
      box.min_x = d.x - d.radius;
      box.max_x = d.x + d.radius;
      box.min_y = d.y - d.radius;
      box.max_y = d.y + d.radius;
    } else {
      box.min_x = std::min(box.min_x, d.x - d.radius);
      box.max_x = std::max(box.max_x, d.x + d.radius);
      box.min_y = std::min(box.min_y, d.y - d.radius);
      box.max_y = std::max(box.max_y, d.y + d.radius);
    }
  }
  return box;
}

std::vector<PaintedCell> rasterise(const std::vector<Disk> & disks, const GridSpec & grid)
{
  std::vector<PaintedCell> cells;
  if (grid.size_x <= 0 || grid.size_y <= 0 || grid.resolution <= 0.0) {
    return cells;
  }
  std::map<std::pair<int, int>, uint8_t> best;
  for (const auto & d : disks) {
    const auto cell_range = [&grid](double lo, double hi, double origin, int size) {
        const double a = std::floor((lo - origin) / grid.resolution);
        const double b = std::floor((hi - origin) / grid.resolution);
        return std::make_pair(
          static_cast<int>(std::clamp(a, 0.0, static_cast<double>(size))),
          static_cast<int>(std::clamp(b, -1.0, static_cast<double>(size - 1))));
      };
    const auto xr = cell_range(d.x - d.radius, d.x + d.radius, grid.origin_x, grid.size_x);
    const auto yr = cell_range(d.y - d.radius, d.y + d.radius, grid.origin_y, grid.size_y);
    const int i0 = xr.first, i1 = xr.second, j0 = yr.first, j1 = yr.second;
    for (int j = j0; j <= j1; ++j) {
      const double cy = grid.origin_y + (j + 0.5) * grid.resolution;
      for (int i = i0; i <= i1; ++i) {
        const double cx = grid.origin_x + (i + 0.5) * grid.resolution;
        if (std::hypot(cx - d.x, cy - d.y) <= d.radius) {
          auto & slot = best[{i, j}];
          slot = std::max(slot, static_cast<uint8_t>(d.cost));
        }
      }
    }
  }
  cells.reserve(best.size());
  for (const auto & entry : best) {
    PaintedCell c;
    c.i = entry.first.first;
    c.j = entry.first.second;
    c.cost = entry.second;
    cells.push_back(c);
  }
  return cells;
}

}  // namespace rapid_nav_hazard
