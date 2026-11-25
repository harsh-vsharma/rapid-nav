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
    const bool uncertain = o.position_sigma > 0.0 || o.velocity_sigma > 0.0;
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
      if (uncertain) {
        d.sigma = std::sqrt(
          o.position_sigma * o.position_sigma + o.velocity_sigma * o.velocity_sigma * t * t);
        d.peak = std::min(cost, static_cast<double>(params.max_cost));
      }
      disks.push_back(d);
    }
  }
  return disks;
}

double normal_cdf(double z)
{
  return 0.5 * (1.0 + std::erf(z / std::sqrt(2.0)));
}

namespace
{
// How far an uncertain disk paints beyond its radius, in sigmas.
constexpr double kReachSigmas = 3.0;

double reach(const Disk & d)
{
  return d.sigma > 0.0 ? d.radius + kReachSigmas * d.sigma : d.radius;
}
}  // namespace

BoundingBox bounding_box(const std::vector<Disk> & disks)
{
  BoundingBox box;
  for (const auto & d : disks) {
    const double r = reach(d);
    if (box.empty) {
      box.empty = false;
      box.min_x = d.x - r;
      box.max_x = d.x + r;
      box.min_y = d.y - r;
      box.max_y = d.y + r;
    } else {
      box.min_x = std::min(box.min_x, d.x - r);
      box.max_x = std::max(box.max_x, d.x + r);
      box.min_y = std::min(box.min_y, d.y - r);
      box.max_y = std::max(box.max_y, d.y + r);
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
    if (d.sigma > 0.0) {
      // Uncertain disk: occupancy probability of the inflated disk, times the time weight.
      const double r = reach(d);
      const auto xr = cell_range(d.x - r, d.x + r, grid.origin_x, grid.size_x);
      const auto yr = cell_range(d.y - r, d.y + r, grid.origin_y, grid.size_y);
      for (int j = yr.first; j <= yr.second; ++j) {
        const double cy = grid.origin_y + (j + 0.5) * grid.resolution;
        for (int i = xr.first; i <= xr.second; ++i) {
          const double cx = grid.origin_x + (i + 0.5) * grid.resolution;
          const double dist = std::hypot(cx - d.x, cy - d.y);
          if (dist > r) {
            continue;
          }
          const double cost = std::min(
            std::round(d.peak * normal_cdf((d.radius - dist) / d.sigma)),
            static_cast<double>(kMaxHazardCost));
          if (cost < 1.0) {
            continue;
          }
          auto & slot = best[{i, j}];
          slot = std::max(slot, static_cast<uint8_t>(cost));
        }
      }
      continue;
    }
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
