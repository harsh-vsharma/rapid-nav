// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).
//
// Grid search for the RAPID-NAV global planner: Dijkstra, A* and Theta* on an 8-connected cost
// grid.
// This header has no ROS dependency; the Nav2 plugin wraps it.

#ifndef RAPID_NAV_PLANNER__GRID_SEARCH_HPP_
#define RAPID_NAV_PLANNER__GRID_SEARCH_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

namespace rapid_nav_planner
{

using Cell = std::pair<int, int>;
using Point = std::pair<double, double>;

enum class Algorithm { kDijkstra, kAStar, kThetaStar };

enum class SearchStatus { kFound, kStartBlocked, kGoalBlocked, kNoPath, kCancelled, kTimedOut };

// Nav2's cost meanings: 0 free, 252 near an obstacle, 253 inscribed, 254 lethal, 255 unknown.
constexpr std::uint8_t kCostUnknown = 255;
constexpr std::uint8_t kCostLethal = 254;
constexpr double kMaxCost = 252.0;

struct SearchParams
{
  Algorithm algorithm = Algorithm::kAStar;
  double cost_weight = 2.0;
  double heuristic_weight = 1.0;
  std::uint8_t lethal_cost = 253;
  bool allow_unknown = true;
  std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::time_point::max();
};

struct SearchResult
{
  SearchStatus status = SearchStatus::kNoPath;
  std::vector<Cell> cells;
  double cost = 0.0;
  std::size_t expansions = 0;
};

class GridView
{
public:
  GridView(int width, int height, const std::uint8_t * costs);

  int width() const { return width_; }
  int height() const { return height_; }
  bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }
  std::uint8_t cost(int x, int y) const { return costs_[static_cast<std::size_t>(y) * width_ + x]; }
  bool blocked(int x, int y, const SearchParams & params) const;

private:
  int width_;
  int height_;
  const std::uint8_t * costs_;
};

// Cost multiplier of entering a cell: 1 + cost_weight * cost / 252; unknown cells count as cost 0.
double cell_multiplier(const GridView & grid, int x, int y, const SearchParams & params);

// True when the straight segment between two cell centres touches no blocked cell. Supercover: a
// segment that
// passes exactly through a corner also checks the two cells beside it.
bool line_of_sight(
  const GridView & grid, int x0, int y0, int x1, int y1, const SearchParams & params);

// Cost of the straight segment between two cell centres: the length inside each cell times that
// cell's
// multiplier, summed.
double line_cost(
  const GridView & grid, int x0, int y0, int x1, int y1, const SearchParams & params);

// Cost of a step between two neighbouring cells: length (1 or sqrt(2)) times the target's
// multiplier.
double step_cost(
  const GridView & grid, int x0, int y0, int x1, int y1, const SearchParams & params);

// Searches for a path from start to goal. The cancel callback and params.deadline are checked every
// 1000
// expansions.
SearchResult search(
  const GridView & grid, Cell start, Cell goal, const SearchParams & params,
  const std::function<bool()> & cancel);

// Greedy shortcutting of a cell path: from each kept point, the farthest later point with a line of
// sight
// whose line cost is at most (1 + tolerance) times the cost of the original sub-path. Returns the
// kept points.
std::vector<Point> smooth(
  const GridView & grid, const std::vector<Cell> & path, const SearchParams & params,
  double tolerance);

// Resamples a polyline so that consecutive points are at most `spacing` apart; keeps the first and
// last.
std::vector<Point> resample(const std::vector<Point> & points, double spacing);

}  // namespace rapid_nav_planner

#endif  // RAPID_NAV_PLANNER__GRID_SEARCH_HPP_
