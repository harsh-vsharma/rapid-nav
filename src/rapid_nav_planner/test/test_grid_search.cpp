// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).
// Tests of the grid search library, with hand-checkable numbers.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <random>
#include <vector>

#include "rapid_nav_planner/grid_search.hpp"

namespace rapid_nav_planner
{
namespace
{

struct Map
{
  int width;
  int height;
  std::vector<std::uint8_t> cells;

  GridView view() const { return GridView(width, height, cells.data()); }
  void set(int x, int y, std::uint8_t value)
  {
    cells[static_cast<std::size_t>(y) * width + x] = value;
  }
};

Map empty_map(int width, int height)
{
  return Map{width, height, std::vector<std::uint8_t>(static_cast<std::size_t>(width) * height, 0)};
}

SearchParams params_for(Algorithm algorithm, double cost_weight = 2.0)
{
  SearchParams params;
  params.algorithm = algorithm;
  params.cost_weight = cost_weight;
  return params;
}

const Algorithm kAllAlgorithms[] = {Algorithm::kDijkstra, Algorithm::kAStar, Algorithm::kThetaStar};

std::function<bool()> never_cancel()
{
  return [] {return false;};
}

SearchResult run(const Map & map, Cell start, Cell goal, const SearchParams & params)
{
  return search(map.view(), start, goal, params, never_cancel());
}

// A grid with random lethal obstacles (cost 0 or 254), seeded so that the tests are repeatable.
Map random_map(std::mt19937 & rng, int size, double obstacle_share)
{
  Map map = empty_map(size, size);
  std::bernoulli_distribution obstacle(obstacle_share);
  for (auto & value : map.cells) {
    value = obstacle(rng) ? 254 : 0;
  }
  return map;
}

Cell random_free_cell(std::mt19937 & rng, const Map & map)
{
  std::uniform_int_distribution<int> any_x(0, map.width - 1);
  std::uniform_int_distribution<int> any_y(0, map.height - 1);
  for (int attempt = 0; attempt < 10000; ++attempt) {
    const Cell cell{any_x(rng), any_y(rng)};
    if (!map.view().blocked(cell.first, cell.second, SearchParams{})) {
      return cell;
    }
  }
  return Cell{0, 0};
}

double path_cost(
  const GridView & grid, const std::vector<Cell> & cells, const SearchParams & params)
{
  double total = 0.0;
  for (std::size_t k = 1; k < cells.size(); ++k) {
    const Cell & a = cells[k - 1];
    const Cell & b = cells[k];
    const bool adjacent = std::max(std::abs(b.first - a.first), std::abs(b.second - a.second)) == 1;
    total += adjacent ? step_cost(grid, a.first, a.second, b.first, b.second, params) :
      line_cost(grid, a.first, a.second, b.first, b.second, params);
  }
  return total;
}

TEST(GridSearch, StraightRowCostsNineteenForEveryAlgorithm)
{
  const Map map = empty_map(20, 20);
  for (const Algorithm algorithm : kAllAlgorithms) {
    const SearchResult result = run(map, {0, 0}, {19, 0}, params_for(algorithm));
    ASSERT_EQ(result.status, SearchStatus::kFound);
    EXPECT_NEAR(result.cost, 19.0, 1e-9);
    if (algorithm == Algorithm::kAStar) {
      EXPECT_EQ(result.cells.size(), 20u);
      for (const Cell & cell : result.cells) {
        EXPECT_EQ(cell.second, 0);
      }
    }
  }
}

TEST(GridSearch, DiagonalCostsTenRootTwo)
{
  const Map map = empty_map(20, 20);
  for (const Algorithm algorithm : kAllAlgorithms) {
    const SearchResult result = run(map, {0, 0}, {10, 10}, params_for(algorithm));
    ASSERT_EQ(result.status, SearchStatus::kFound);
    EXPECT_NEAR(result.cost, 10.0 * std::sqrt(2.0), 1e-9);
  }
}

TEST(GridSearch, PathPassesTheGapInAWall)
{
  Map map = empty_map(20, 20);
  for (int y = 0; y < 19; ++y) {
    map.set(10, y, 254);
  }
  const SearchResult result = run(map, {0, 0}, {19, 0}, params_for(Algorithm::kAStar));
  ASSERT_EQ(result.status, SearchStatus::kFound);
  bool through_gap = false;
  for (const Cell & cell : result.cells) {
    through_gap = through_gap || cell == Cell{10, 19};
  }
  EXPECT_TRUE(through_gap);
}

TEST(GridSearch, NoCornerCuttingBetweenTwoBlockedCells)
{
  Map map = empty_map(3, 3);
  map.set(1, 0, 254);
  map.set(0, 1, 254);
  for (const Algorithm algorithm : kAllAlgorithms) {
    EXPECT_EQ(run(map, {0, 0}, {1, 1}, params_for(algorithm)).status, SearchStatus::kNoPath);
  }
}

TEST(GridSearch, BlockedStartGoalAndEnclosedGoal)
{
  Map map = empty_map(10, 10);
  map.set(5, 5, 254);
  const SearchParams astar = params_for(Algorithm::kAStar);
  EXPECT_EQ(run(map, {5, 5}, {0, 0}, astar).status, SearchStatus::kStartBlocked);
  EXPECT_EQ(run(map, {0, 0}, {5, 5}, astar).status, SearchStatus::kGoalBlocked);

  Map enclosed = empty_map(10, 10);
  for (int y = 3; y <= 7; ++y) {
    for (int x = 3; x <= 7; ++x) {
      if (x == 3 || x == 7 || y == 3 || y == 7) {
        enclosed.set(x, y, 254);
      }
    }
  }
  EXPECT_EQ(run(enclosed, {0, 0}, {5, 5}, params_for(Algorithm::kAStar)).status,
    SearchStatus::kNoPath);
}

TEST(GridSearch, DijkstraAndAStarAgreeAndAStarExpandsNoMore)
{
  std::mt19937 rng(20261112);
  for (int trial = 0; trial < 50; ++trial) {
    const Map map = random_map(rng, 60, 0.25);
    const Cell start = random_free_cell(rng, map);
    const Cell goal = random_free_cell(rng, map);
    const SearchResult dijkstra = run(map, start, goal, params_for(Algorithm::kDijkstra));
    const SearchResult astar = run(map, start, goal, params_for(Algorithm::kAStar));
    ASSERT_EQ(dijkstra.status, astar.status) << "trial " << trial;
    if (dijkstra.status == SearchStatus::kFound) {
      EXPECT_NEAR(dijkstra.cost, astar.cost, 1e-9) << "trial " << trial;
      EXPECT_LE(astar.expansions, dijkstra.expansions) << "trial " << trial;
    }
  }
}

TEST(GridSearch, CostWeightMakesThePlannerAvoidExpensiveCells)
{
  // A row of cost 200 along y = 10 from x = 0 to 29; the free detour is one row further.
  Map map = empty_map(30, 21);
  for (int x = 0; x < 30; ++x) {
    map.set(x, 10, 200);
  }
  const SearchResult cheap = run(map, {0, 10}, {29, 10}, params_for(Algorithm::kAStar, 0.0));
  ASSERT_EQ(cheap.status, SearchStatus::kFound);
  for (const Cell & cell : cheap.cells) {
    EXPECT_EQ(cell.second, 10);
  }
  const SearchResult detour = run(map, {0, 10}, {29, 10}, params_for(Algorithm::kAStar, 4.0));
  ASSERT_EQ(detour.status, SearchStatus::kFound);
  bool off_the_row = false;
  for (const Cell & cell : detour.cells) {
    off_the_row = off_the_row || cell.second != 10;
  }
  EXPECT_TRUE(off_the_row);
}

TEST(GridSearch, ThetaStarMakesOneStraightLineOnAnEmptyGrid)
{
  const Map map = empty_map(20, 20);
  const SearchResult result = run(map, {0, 0}, {10, 3}, params_for(Algorithm::kThetaStar));
  ASSERT_EQ(result.status, SearchStatus::kFound);
  EXPECT_EQ(result.cells.size(), 2u);
  EXPECT_NEAR(result.cost, std::sqrt(109.0), 1e-9);
}

TEST(GridSearch, ThetaStarIsNoLongerThanAStarAndSegmentsSeeEachOther)
{
  std::mt19937 rng(777);
  for (int trial = 0; trial < 50; ++trial) {
    const Map map = random_map(rng, 60, 0.25);
    const Cell start = random_free_cell(rng, map);
    const Cell goal = random_free_cell(rng, map);
    const SearchResult astar = run(map, start, goal, params_for(Algorithm::kAStar));
    const SearchResult theta = run(map, start, goal, params_for(Algorithm::kThetaStar));
    ASSERT_EQ(astar.status, theta.status) << "trial " << trial;
    if (theta.status != SearchStatus::kFound) {
      continue;
    }
    EXPECT_LE(theta.cost, astar.cost + 1e-9) << "trial " << trial;
    for (std::size_t k = 1; k < theta.cells.size(); ++k) {
      const Cell & a = theta.cells[k - 1];
      const Cell & b = theta.cells[k];
      EXPECT_TRUE(line_of_sight(
          map.view(), a.first, a.second, b.first, b.second, params_for(Algorithm::kAStar)))
        << "trial " << trial << " segment " << k;
    }
  }
}

TEST(GridSearch, LineOfSightThroughCornersAndAlongWalls)
{
  const SearchParams params = params_for(Algorithm::kAStar);
  Map corner = empty_map(5, 5);
  corner.set(1, 0, 254);
  corner.set(0, 1, 254);
  EXPECT_FALSE(line_of_sight(corner.view(), 0, 0, 1, 1, params));

  Map wall = empty_map(6, 6);
  for (int x = 0; x < 5; ++x) {
    wall.set(x, 2, 254);
  }
  EXPECT_TRUE(line_of_sight(wall.view(), 0, 1, 4, 1, params));
  EXPECT_FALSE(line_of_sight(wall.view(), 0, 1, 4, 3, params));
  EXPECT_TRUE(line_of_sight(empty_map(6, 6).view(), 0, 0, 5, 3, params));
}

TEST(GridSearch, SmoothingKeepsSegmentsFreeAndTheCostWithinTolerance)
{
  std::mt19937 rng(4242);
  for (int trial = 0; trial < 20; ++trial) {
    const Map map = random_map(rng, 60, 0.25);
    const Cell start = random_free_cell(rng, map);
    const Cell goal = random_free_cell(rng, map);
    const SearchParams params = params_for(Algorithm::kAStar);
    const SearchResult result = run(map, start, goal, params);
    if (result.status != SearchStatus::kFound) {
      continue;
    }
    // On a grid path the cost of the moves equals the search's cost.
    EXPECT_NEAR(path_cost(map.view(), result.cells, params), result.cost, 1e-9)
      << "trial " << trial;
    const std::vector<Point> smoothed = smooth(map.view(), result.cells, params, 0.1);
    ASSERT_GE(smoothed.size(), 2u);
    EXPECT_EQ(smoothed.front(), Point(start.first, start.second));
    EXPECT_EQ(smoothed.back(), Point(goal.first, goal.second));
    std::vector<Cell> kept;
    for (const Point & point : smoothed) {
      kept.emplace_back(static_cast<int>(point.first), static_cast<int>(point.second));
    }
    for (std::size_t k = 1; k < kept.size(); ++k) {
      EXPECT_TRUE(line_of_sight(
          map.view(), kept[k - 1].first, kept[k - 1].second, kept[k].first, kept[k].second, params))
        << "trial " << trial << " segment " << k;
    }
    EXPECT_LE(path_cost(map.view(), kept, params), 1.1 * result.cost + 1e-9) << "trial " << trial;
  }
}

TEST(GridSearch, CancelAndDeadlineStopTheSearch)
{
  const Map map = empty_map(50, 50);
  const SearchResult cancelled = search(
    map.view(), {0, 0}, {49, 49}, params_for(Algorithm::kAStar), [] {return true;});
  EXPECT_EQ(cancelled.status, SearchStatus::kCancelled);

  SearchParams late = params_for(Algorithm::kAStar);
  late.deadline = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  EXPECT_EQ(run(map, {0, 0}, {49, 49}, late).status, SearchStatus::kTimedOut);
}

TEST(GridSearch, ResampleSpacesPointsEvenly)
{
  const std::vector<Point> points = {{0.0, 0.0}, {3.0, 0.0}};
  const std::vector<Point> spaced = resample(points, 1.0);
  ASSERT_EQ(spaced.size(), 4u);
  for (std::size_t k = 0; k < spaced.size(); ++k) {
    EXPECT_NEAR(spaced[k].first, static_cast<double>(k), 1e-12);
  }
}

}  // namespace
}  // namespace rapid_nav_planner
