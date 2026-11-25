// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>
#include <vector>

#include "rapid_nav_hazard/hazard_core.hpp"

using rapid_nav_hazard::bounding_box;
using rapid_nav_hazard::Disk;
using rapid_nav_hazard::GridSpec;
using rapid_nav_hazard::HazardParams;
using rapid_nav_hazard::kMaxHazardCost;
using rapid_nav_hazard::normal_cdf;
using rapid_nav_hazard::Obstacle;
using rapid_nav_hazard::predict_disks;
using rapid_nav_hazard::rasterise;

namespace
{
Obstacle make(double x, double y, double vx, double vy, double r)
{
  Obstacle o;
  o.x = x;
  o.y = y;
  o.vx = vx;
  o.vy = vy;
  o.radius = r;
  return o;
}

Obstacle uncertain(double x, double y, double vx, double vy, double r, double sp, double sv)
{
  Obstacle o = make(x, y, vx, vy, r);
  o.position_sigma = sp;
  o.velocity_sigma = sv;
  return o;
}

uint8_t peak_of(const std::vector<rapid_nav_hazard::PaintedCell> & cells)
{
  uint8_t best = 0;
  for (const auto & c : cells) {
    best = std::max(best, c.cost);
  }
  return best;
}

GridSpec grid()
{
  GridSpec g;
  g.origin_x = -5.0;
  g.origin_y = -5.0;
  g.resolution = 0.05;
  g.size_x = 200;
  g.size_y = 200;
  return g;
}
}  // namespace

TEST(HazardCore, StillObstacleIsOneDisk)
{
  const HazardParams p;
  const auto disks = predict_disks({make(1.0, 2.0, 0.0, 0.0, 0.2)}, 0.0, p);
  ASSERT_EQ(disks.size(), 1u);
  EXPECT_DOUBLE_EQ(disks[0].x, 1.0);
  EXPECT_DOUBLE_EQ(disks[0].y, 2.0);
  EXPECT_NEAR(disks[0].radius, 0.2 + 0.25 + 0.05, 1e-12);
  EXPECT_EQ(disks[0].cost, 252);
  const auto cells = rasterise(disks, grid());
  ASSERT_FALSE(cells.empty());
  // Disk radius 0.5 m: pi r^2 = 0.785 m^2 = 314 cells of 0.0025 m^2.
  EXPECT_NEAR(static_cast<double>(cells.size()), 314.2, 12.0);
  for (const auto & c : cells) {
    EXPECT_EQ(c.cost, 252);
  }
}

TEST(HazardCore, MovingObstacleMakesABand)
{
  const HazardParams p;
  const auto disks = predict_disks({make(0.0, 0.0, 0.5, 0.0, 0.2)}, 0.0, p);
  ASSERT_EQ(disks.size(), 9u);  // k = 0 .. 8
  EXPECT_EQ(disks.front().cost, 252);
  EXPECT_EQ(disks.back().cost, 60);
  EXPECT_NEAR(disks.back().x, 0.5 * 2.0, 1e-12);
  // Cost falls monotonically along the band.
  for (size_t k = 1; k < disks.size(); ++k) {
    EXPECT_LT(disks[k].cost, disks[k - 1].cost);
  }
  const auto box = bounding_box(disks);
  EXPECT_FALSE(box.empty);
  EXPECT_NEAR(box.max_x - box.min_x, 1.0 + 2 * 0.5, 1e-12);
  // The cell at the far tip of the band (1.0 + 0.5 m) is covered by the last disk only: cost 60.
  const auto cells = rasterise(disks, grid());
  uint8_t far_cost = 0;
  for (const auto & c : cells) {
    const double cx = -5.0 + (c.i + 0.5) * 0.05;
    const double cy = -5.0 + (c.j + 0.5) * 0.05;
    if (std::hypot(cx - 1.49, cy) < 0.03) {
      far_cost = c.cost;
    }
  }
  EXPECT_EQ(far_cost, 60);
}

TEST(HazardCore, MinSpeedCutoff)
{
  const HazardParams p;
  const auto disks = predict_disks({make(1.0, 1.0, 0.04, 0.0, 0.2)}, 0.3, p);
  ASSERT_EQ(disks.size(), 1u);
  EXPECT_DOUBLE_EQ(disks[0].x, 1.0);
  const auto moving = predict_disks({make(1.0, 1.0, 0.05, 0.0, 0.2)}, 0.0, p);
  EXPECT_EQ(moving.size(), 9u);
}

TEST(HazardCore, AgeShiftsTheFirstDisk)
{
  const HazardParams p;
  const auto disks = predict_disks({make(0.0, 0.0, 0.5, -0.25, 0.2)}, 0.4, p);
  EXPECT_NEAR(disks[0].x, 0.2, 1e-12);
  EXPECT_NEAR(disks[0].y, -0.1, 1e-12);
  EXPECT_EQ(disks[0].cost, 252);
}

TEST(HazardCore, OverlapTakesTheMaximum)
{
  const HazardParams p;
  // A still obstacle (cost 252) sits on the far end of a mover's band (cost 60 there).
  const auto disks = predict_disks(
    {make(0.0, 0.0, 0.5, 0.0, 0.2), make(1.0, 0.0, 0.0, 0.0, 0.2)}, 0.0, p);
  const auto cells = rasterise(disks, grid());
  for (const auto & c : cells) {
    const double cx = -5.0 + (c.i + 0.5) * 0.05;
    const double cy = -5.0 + (c.j + 0.5) * 0.05;
    if (std::hypot(cx - 1.0, cy) < 0.02) {
      EXPECT_EQ(c.cost, 252);
    }
  }
}

TEST(HazardCore, NothingAboveMaxCost)
{
  HazardParams p;
  p.max_cost = 300;  // the core never exceeds the cost it is given
  const auto disks = predict_disks({make(0.0, 0.0, 0.5, 0.0, 0.2)}, 0.0, p);
  for (const auto & d : disks) {
    EXPECT_LE(d.cost, p.max_cost);
  }
  const HazardParams q;
  const auto cells = rasterise(predict_disks({make(0.0, 0.0, 0.5, 0.3, 0.4)}, 1.0, q), grid());
  for (const auto & c : cells) {
    EXPECT_LE(c.cost, 252);
    EXPECT_GE(c.cost, 60);
  }
}

TEST(HazardCore, OutsideTheGridPaintsNothing)
{
  const HazardParams p;
  EXPECT_TRUE(rasterise(predict_disks({make(50.0, 50.0, 0.5, 0.0, 0.2)}, 0.0, p), grid()).empty());
  EXPECT_TRUE(rasterise(predict_disks({make(-50.0, 0.0, 0.0, 0.0, 0.2)}, 0.0, p), grid()).empty());
  const auto still = predict_disks({make(0.0, 0.0, 0.0, 0.0, 0.2)}, 0.0, p);
  EXPECT_TRUE(rasterise(still, GridSpec()).empty());
  // A band that starts inside and leaves the grid is clipped, not an error.
  const auto cells = rasterise(predict_disks({make(4.9, 0.0, 1.0, 0.0, 0.2)}, 0.0, p), grid());
  EXPECT_FALSE(cells.empty());
  for (const auto & c : cells) {
    EXPECT_LT(c.i, 200);
    EXPECT_GE(c.i, 0);
  }
}

TEST(HazardCore, NoObstaclesNoBox)
{
  const HazardParams p;
  const auto disks = predict_disks({}, 0.0, p);
  EXPECT_TRUE(disks.empty());
  EXPECT_TRUE(bounding_box(disks).empty);
}

// The S7 rasterisation, written out independently: hard disks, maximum cost per cell centre.
TEST(HazardCore, ZeroSigmaIsTheS7OutputBitForBit)
{
  const HazardParams p;
  const std::vector<Obstacle> obstacles = {
    make(0.0, 0.0, 0.5, 0.0, 0.2), make(1.0, 0.3, 0.0, 0.0, 0.3), make(-1.2, 0.8, -0.2, 0.6, 0.25),
    uncertain(2.0, -1.0, 0.4, 0.4, 0.2, 0.0, 0.0)};
  const auto disks = predict_disks(obstacles, 0.2, p);
  for (const auto & d : disks) {
    EXPECT_EQ(d.sigma, 0.0);
  }
  const GridSpec g = grid();
  std::map<std::pair<int, int>, int> expected;
  for (const auto & d : disks) {
    for (int j = 0; j < g.size_y; ++j) {
      for (int i = 0; i < g.size_x; ++i) {
        const double cx = g.origin_x + (i + 0.5) * g.resolution;
        const double cy = g.origin_y + (j + 0.5) * g.resolution;
        if (std::hypot(cx - d.x, cy - d.y) <= d.radius) {
          auto & slot = expected[{i, j}];
          slot = std::max(slot, d.cost);
        }
      }
    }
  }
  const auto cells = rasterise(disks, g);
  ASSERT_EQ(cells.size(), expected.size());
  for (const auto & c : cells) {
    EXPECT_EQ(static_cast<int>(c.cost), expected.at({c.i, c.j}));
  }
}

TEST(HazardCore, UncertainDisksCarrySigmaAndPeak)
{
  const HazardParams p;
  const auto disks = predict_disks({uncertain(0.0, 0.0, 0.5, 0.0, 0.2, 0.1, 0.2)}, 0.0, p);
  ASSERT_EQ(disks.size(), 9u);
  for (size_t k = 0; k < disks.size(); ++k) {
    const double t = 0.25 * k;
    EXPECT_NEAR(disks[k].sigma, std::sqrt(0.01 + 0.04 * t * t), 1e-12);
    EXPECT_NEAR(disks[k].peak, 252.0 - (252.0 - 60.0) * t / 2.0, 1e-9);
  }
  EXPECT_NEAR(normal_cdf(0.0), 0.5, 1e-12);
  EXPECT_NEAR(normal_cdf(1.0), 0.841345, 1e-6);
}

TEST(HazardCore, LargerSigmaGivesALowerPeakAndAWiderFootprint)
{
  const HazardParams p;
  size_t last_size = 0;
  int last_peak = 253;
  for (double sigma : {0.2, 0.3, 0.5, 0.8}) {
    const auto disks = predict_disks({uncertain(0.0, 0.0, 0.0, 0.0, 0.2, sigma, 0.0)}, 0.0, p);
    const auto cells = rasterise(disks, grid());
    const int peak = peak_of(cells);
    EXPECT_LT(peak, last_peak) << "sigma " << sigma;
    EXPECT_GT(cells.size(), last_size) << "sigma " << sigma;
    // Centre cell: P = Phi(R / sigma).
    EXPECT_NEAR(peak, 252.0 * normal_cdf(0.5 / sigma), 1.0 + 252.0 * 0.03 / sigma);
    last_peak = peak;
    last_size = cells.size();
  }
  // A small sigma is close to the hard disk: almost 252 at the centre, 126 at the edge.
  const auto sharp = rasterise(
    predict_disks({uncertain(0.0, 0.0, 0.0, 0.0, 0.2, 0.01, 0.0)}, 0.0, p), grid());
  EXPECT_EQ(peak_of(sharp), 252);
  // The velocity sigma widens the far disks only: k = 0 has sigma = position_sigma.
  const auto band = predict_disks({uncertain(0.0, 0.0, 0.5, 0.0, 0.2, 0.05, 0.4)}, 0.0, p);
  EXPECT_NEAR(band.front().sigma, 0.05, 1e-12);
  EXPECT_NEAR(band.back().sigma, std::sqrt(0.05 * 0.05 + 0.16 * 4.0), 1e-12);
  // The box reaches 3 sigma beyond the radius.
  const auto box = bounding_box(band);
  EXPECT_NEAR(box.max_x, 1.0 + 0.5 + 3.0 * band.back().sigma, 1e-9);
}

TEST(HazardCore, UncertainCostNeverReachesTheInscribedValue)
{
  HazardParams p;
  for (int max_cost : {252, 300}) {
    p.max_cost = max_cost;
    for (double sp : {0.0, 0.001, 0.05, 0.5}) {
      for (double sv : {0.001, 0.1, 1.0}) {
        const auto cells = rasterise(
          predict_disks({uncertain(0.0, 0.0, 0.5, 0.2, 0.3, sp, sv)}, 0.1, p), grid());
        ASSERT_FALSE(cells.empty());
        for (const auto & c : cells) {
          EXPECT_LE(static_cast<int>(c.cost), kMaxHazardCost);
          EXPECT_GE(static_cast<int>(c.cost), 1);
        }
      }
    }
  }
}
