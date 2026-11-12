// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).
// Tests of the benchmark functions on a small built-in room (not the command line).

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include "rapid_nav_planner/bench.hpp"

namespace rapid_nav_planner
{
namespace
{

constexpr int kWidth = 30;
constexpr int kHeight = 30;
constexpr int kAlgorithms = 4;
constexpr int kGapFirstRow = 6;  // the 0.9 m gap in the wall at column 20 is rows 6 to 23
constexpr int kGapLastRow = 23;

// A 30 x 30 room of 0.05 m cells with a wall at column 20 that has a 0.9 m gap.
BenchGrid room()
{
  std::vector<std::uint8_t> occupied(kWidth * kHeight, 0);
  for (int y = 0; y < kHeight; ++y) {
    if (y < kGapFirstRow || y > kGapLastRow) {
      occupied[static_cast<std::size_t>(y) * kWidth + 20] = 1;
    }
  }
  return build_cost_grid(occupied, kWidth, kHeight, 0.05, 0.0, 0.0, 0.25);
}

std::uint8_t cost_at(const BenchGrid & grid, int x, int y)
{
  return grid.costs[static_cast<std::size_t>(y) * grid.width + x];
}

TEST(BenchGrid, CostsFollowTheInflationRule)
{
  const BenchGrid grid = room();
  EXPECT_EQ(cost_at(grid, 20, 5), kCostLethal);  // the wall
  EXPECT_EQ(cost_at(grid, 19, 5), 253);  // next to the wall: inside the robot radius
  EXPECT_EQ(cost_at(grid, 14, 5), 217);  // 0.30 m out: 252 * exp(-3 * 0.05)
  EXPECT_EQ(cost_at(grid, 0, 5), 0);  // more than 0.55 m from the wall
}

TEST(BenchGrid, RandomQueriesAreFreeDistinctAndReproducible)
{
  const BenchGrid grid = room();
  const std::vector<BenchQuery> first = random_queries(grid, 20, 7);
  const std::vector<BenchQuery> second = random_queries(grid, 20, 7);
  ASSERT_EQ(first.size(), 20u);
  EXPECT_EQ(first.front().name, "q0001");
  for (std::size_t k = 0; k < first.size(); ++k) {
    EXPECT_EQ(first[k].start, second[k].start);
    EXPECT_EQ(first[k].goal, second[k].goal);
    EXPECT_NE(first[k].start, first[k].goal);
    EXPECT_LT(cost_at(grid, first[k].start.first, first[k].start.second), 253);
    EXPECT_LT(cost_at(grid, first[k].goal.first, first[k].goal.second), 253);
  }
}

TEST(BenchGrid, QueriesStayInTheLargestRegion)
{
  // A full-height wall at column 30 splits a 40 x 10 grid: a large room on the left, a small one
  // on the right. Every query must lie in the large room.
  std::vector<std::uint8_t> occupied(40 * 10, 0);
  for (int y = 0; y < 10; ++y) {
    occupied[static_cast<std::size_t>(y) * 40 + 30] = 1;
  }
  const BenchGrid grid = build_cost_grid(occupied, 40, 10, 0.05, 0.0, 0.0, 0.25);
  const std::vector<BenchQuery> queries = random_queries(grid, 50, 11);
  ASSERT_EQ(queries.size(), 50u);
  for (const BenchQuery & query : queries) {
    EXPECT_LT(query.start.first, 30) << query.name;
    EXPECT_LT(query.goal.first, 30) << query.name;
  }
}

TEST(BenchRun, DijkstraAndAStarFindTheSameOptimalCost)
{
  const BenchGrid grid = room();
  const std::vector<BenchQuery> queries = random_queries(grid, 25, 3);
  const std::vector<BenchRow> rows = run_bench(grid, queries, default_algorithms(2.0));
  ASSERT_EQ(rows.size(), queries.size() * kAlgorithms);
  for (std::size_t q = 0; q < queries.size(); ++q) {
    const BenchRow & dijkstra = rows[q * kAlgorithms + 0];
    const BenchRow & astar = rows[q * kAlgorithms + 1];
    ASSERT_EQ(dijkstra.algorithm, "dijkstra");
    ASSERT_EQ(astar.algorithm, "astar");
    EXPECT_EQ(dijkstra.status, astar.status);
    if (dijkstra.status == "found") {
      EXPECT_NEAR(dijkstra.cost, astar.cost, 1e-6) << queries[q].name;
      EXPECT_LE(astar.expansions, dijkstra.expansions) << queries[q].name;
    }
  }
}

TEST(BenchRun, PathsGoThroughTheGapAndCsvHasOneLinePerSearch)
{
  const BenchGrid grid = room();
  // Both ends are above the wall: the way round is down to the gap and back up.
  const BenchQuery across{"across", Cell(2, 27), Cell(27, 27)};
  const std::vector<BenchRow> rows = run_bench(grid, {across}, default_algorithms(2.0));
  ASSERT_EQ(rows.size(), static_cast<std::size_t>(kAlgorithms));
  for (const BenchRow & row : rows) {
    EXPECT_EQ(row.status, "found") << row.algorithm;
    // A straight line would cross the wall, so the path is longer than the 25 cells (1.25 m).
    EXPECT_GT(row.length_m, 1.25) << row.algorithm;
  }

  std::ostringstream csv;
  write_csv(csv, rows);
  std::istringstream lines(csv.str());
  std::string line;
  std::getline(lines, line);
  EXPECT_EQ(line, "query,algorithm,status,cost,length_m,expansions,time_us");
  std::size_t count = 0;
  while (std::getline(lines, line)) {
    ++count;
  }
  EXPECT_EQ(count, rows.size());  // one line per search after the header
}

}  // namespace
}  // namespace rapid_nav_planner
