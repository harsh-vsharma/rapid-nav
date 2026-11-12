// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).

// Benchmark of the grid search, without ROS: load a map, build the cost grid the planner sees, draw
// random start and goal cells, time Dijkstra, A*, weighted A* and Theta* on each query.

#ifndef RAPID_NAV_PLANNER__BENCH_HPP_
#define RAPID_NAV_PLANNER__BENCH_HPP_

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <utility>
#include <vector>

#include "rapid_nav_planner/grid_search.hpp"

namespace rapid_nav_planner
{

// The cost grid the planner searches: row-major, row 0 at the bottom, the origin is the lower-left
// corner of the lower-left cell.
struct BenchGrid
{
  int width = 0;
  int height = 0;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<std::uint8_t> costs;
};

struct BenchQuery
{
  std::string name;
  Cell start;
  Cell goal;
};

struct BenchAlgorithm
{
  std::string name;
  SearchParams params;
};

struct BenchRow
{
  std::string query;
  std::string algorithm;
  std::string status;
  double cost = 0.0;
  double length_m = 0.0;
  std::size_t expansions = 0;
  std::int64_t time_us = 0;
};

// Lethal cost of occupied cells, the inscribed cost of cells within the robot radius, and the
// decay beyond it: cost = 252 * exp(-3 * (d - inflate_radius)) for d up to 0.55 m, where d is the
// distance in metres to the nearest occupied cell centre. This is Nav2's inflation layer
// (cost_scaling_factor 3.0, inflation_radius 0.55 m) with the inscribed radius as a parameter.
BenchGrid build_cost_grid(
  const std::vector<std::uint8_t> & occupied, int width, int height, double resolution,
  double origin_x, double origin_y, double inflate_radius);

// Load a trinary map (the YAML written by make_maps.py and read by map_server) and build its grid.
// Returns an empty string on success, otherwise the reason it failed.
std::string load_grid(const std::string & map_yaml, double inflate_radius, BenchGrid & grid);

// Start and goal cells drawn uniformly, each pair with distinct cells, from the largest region of
// free cells: the cells the search can reach from one another. The map's free space outside the
// walls is not in that region, so it gives no queries. The same seed gives the same queries.
std::vector<BenchQuery> random_queries(const BenchGrid & grid, int count, unsigned int seed);

// Dijkstra, A*, A* with heuristic weight 1.5, and Theta*; all with the planner's cost weight.
std::vector<BenchAlgorithm> default_algorithms(double cost_weight);

// Run every algorithm on every query and time each search with the steady clock.
std::vector<BenchRow> run_bench(
  const BenchGrid & grid, const std::vector<BenchQuery> & queries,
  const std::vector<BenchAlgorithm> & algorithms);

// Write the header and one line per row: query,algorithm,status,cost,length_m,expansions,time_us.
// Cost and length are nan when the search found no path.
void write_csv(std::ostream & out, const std::vector<BenchRow> & rows);

}  // namespace rapid_nav_planner

#endif  // RAPID_NAV_PLANNER__BENCH_HPP_
