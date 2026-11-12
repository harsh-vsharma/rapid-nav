// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).
// Grid search for the RAPID-NAV global planner (see grid_search.hpp).

#include "rapid_nav_planner/grid_search.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iterator>
#include <limits>
#include <queue>

namespace rapid_nav_planner
{
namespace
{

constexpr double kSqrt2 = 1.4142135623730951;
constexpr double kTiny = 1e-12;
constexpr std::size_t kCheckEvery = 1000;
constexpr int kDirections[8][2] = {
  {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

double octile_distance(int x0, int y0, int x1, int y1)
{
  const double dx = std::abs(x1 - x0);
  const double dy = std::abs(y1 - y0);
  return std::max(dx, dy) + (kSqrt2 - 1.0) * std::min(dx, dy);
}

bool adjacent(int x0, int y0, int x1, int y1)
{
  return std::max(std::abs(x1 - x0), std::abs(y1 - y0)) == 1;
}

}  // namespace

GridView::GridView(int width, int height, const std::uint8_t * costs)
: width_(width), height_(height), costs_(costs)
{
}

bool GridView::blocked(int x, int y, const SearchParams & params) const
{
  if (!inside(x, y)) {
    return true;
  }
  const std::uint8_t value = cost(x, y);
  if (value == kCostUnknown) {
    return !params.allow_unknown;
  }
  return value >= params.lethal_cost;
}

double cell_multiplier(const GridView & grid, int x, int y, const SearchParams & params)
{
  const std::uint8_t value = grid.cost(x, y);
  if (value == kCostUnknown) {
    return 1.0;
  }
  return 1.0 + params.cost_weight * static_cast<double>(value) / kMaxCost;
}

bool line_of_sight(
  const GridView & grid, int x0, int y0, int x1, int y1, const SearchParams & params)
{
  if (grid.blocked(x0, y0, params) || grid.blocked(x1, y1, params)) {
    return false;
  }
  // Supercover Bresenham: every cell the segment touches, including both cells beside a corner.
  const int dx = x1 - x0;
  const int dy = y1 - y0;
  const int steps_x = std::abs(dx);
  const int steps_y = std::abs(dy);
  const int sign_x = dx >= 0 ? 1 : -1;
  const int sign_y = dy >= 0 ? 1 : -1;
  int x = x0;
  int y = y0;
  int done_x = 0;
  int done_y = 0;
  while (done_x < steps_x || done_y < steps_y) {
    const int decision = (1 + 2 * done_x) * steps_y - (1 + 2 * done_y) * steps_x;
    if (decision == 0) {
      if (grid.blocked(x + sign_x, y, params) || grid.blocked(x, y + sign_y, params)) {
        return false;
      }
      x += sign_x;
      y += sign_y;
      ++done_x;
      ++done_y;
    } else if (decision < 0) {
      x += sign_x;
      ++done_x;
    } else {
      y += sign_y;
      ++done_y;
    }
    if (grid.blocked(x, y, params)) {
      return false;
    }
  }
  return true;
}

void axis_breaks(double from, double to, std::vector<double> & out)
{
  // Parameters t in (0, 1) at which a segment crosses a cell border on one axis, in
  // increasing order.
  const double delta = to - from;
  if (delta == 0.0) {
    return;
  }
  const double low = std::min(from, to);
  const double high = std::max(from, to);
  if (delta > 0.0) {
    for (double border = std::floor(low) + 0.5; border < high; border += 1.0) {
      if (border > low) {
        out.push_back((border - from) / delta);
      }
    }
  } else {
    for (double border = std::ceil(high) - 0.5; border > low; border -= 1.0) {
      if (border < high) {
        out.push_back((border - from) / delta);
      }
    }
  }
}

double line_cost(
  const GridView & grid, int x0, int y0, int x1, int y1, const SearchParams & params)
{
  const double dx = static_cast<double>(x1 - x0);
  const double dy = static_cast<double>(y1 - y0);
  const double length = std::hypot(dx, dy);
  if (length < kTiny) {
    return 0.0;
  }
  std::vector<double> across_x;
  std::vector<double> across_y;
  across_x.reserve(static_cast<std::size_t>(std::abs(x1 - x0)));
  across_y.reserve(static_cast<std::size_t>(std::abs(y1 - y0)));
  axis_breaks(x0, x1, across_x);
  axis_breaks(y0, y1, across_y);
  std::vector<double> breaks;
  breaks.reserve(across_x.size() + across_y.size() + 2);
  breaks.push_back(0.0);
  std::merge(across_x.begin(), across_x.end(), across_y.begin(), across_y.end(),
    std::back_inserter(breaks));
  breaks.push_back(1.0);

  double cost = 0.0;
  for (std::size_t i = 0; i + 1 < breaks.size(); ++i) {
    const double a = breaks[i];
    const double b = breaks[i + 1];
    if (b - a <= kTiny) {
      continue;
    }
    const double middle = 0.5 * (a + b);
    const int cell_x = static_cast<int>(std::floor(x0 + middle * dx + 0.5));
    const int cell_y = static_cast<int>(std::floor(y0 + middle * dy + 0.5));
    cost += length * (b - a) * cell_multiplier(grid, cell_x, cell_y, params);
  }
  return cost;
}

double step_cost(const GridView & grid, int x0, int y0, int x1, int y1, const SearchParams & params)
{
  const double length = (x0 != x1 && y0 != y1) ? kSqrt2 : 1.0;
  return length * cell_multiplier(grid, x1, y1, params);
}

SearchResult search(
  const GridView & grid, Cell start, Cell goal, const SearchParams & params,
  const std::function<bool()> & cancel)
{
  SearchResult result;
  if (!grid.inside(start.first, start.second) || grid.blocked(start.first, start.second, params)) {
    result.status = SearchStatus::kStartBlocked;
    return result;
  }
  if (!grid.inside(goal.first, goal.second) || grid.blocked(goal.first, goal.second, params)) {
    result.status = SearchStatus::kGoalBlocked;
    return result;
  }
  if (start == goal) {
    result.status = SearchStatus::kFound;
    result.cells = {start};
    return result;
  }

  const int width = grid.width();
  const std::size_t count = static_cast<std::size_t>(grid.width()) * grid.height();
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> best(count, infinity);
  std::vector<int> parent(count, -1);
  std::vector<char> closed(count, 0);
  const auto index_of = [width](int x, int y) { return y * width + x; };
  const bool use_heuristic = params.algorithm != Algorithm::kDijkstra;
  const auto heuristic = [&](int x, int y) {
      if (!use_heuristic) {
        return 0.0;
      }
      return params.heuristic_weight * octile_distance(x, y, goal.first, goal.second);
    };

  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  const int start_index = index_of(start.first, start.second);
  const int goal_index = index_of(goal.first, goal.second);
  best[start_index] = 0.0;
  open.emplace(heuristic(start.first, start.second), start_index);

  std::size_t next_check = 0;
  while (!open.empty()) {
    if (result.expansions >= next_check) {
      next_check = result.expansions + kCheckEvery;
      if (cancel && cancel()) {
        result.status = SearchStatus::kCancelled;
        return result;
      }
      if (std::chrono::steady_clock::now() > params.deadline) {
        result.status = SearchStatus::kTimedOut;
        return result;
      }
    }
    const int current = open.top().second;
    open.pop();
    if (closed[current]) {
      continue;
    }
    closed[current] = 1;
    ++result.expansions;
    if (current == goal_index) {
      result.status = SearchStatus::kFound;
      result.cost = best[current];
      for (int node = current; node != -1; node = parent[node]) {
        result.cells.emplace_back(node % width, node / width);
      }
      std::reverse(result.cells.begin(), result.cells.end());
      return result;
    }

    const int cx = current % width;
    const int cy = current / width;
    for (const auto & direction : kDirections) {
      const int nx = cx + direction[0];
      const int ny = cy + direction[1];
      if (grid.blocked(nx, ny, params)) {
        continue;
      }
      const bool diagonal = direction[0] != 0 && direction[1] != 0;
      if (diagonal && (grid.blocked(cx + direction[0], cy, params) ||
        grid.blocked(cx, cy + direction[1], params)))
      {
        continue;  // no corner cutting
      }
      const int neighbour = index_of(nx, ny);
      if (closed[neighbour]) {
        continue;
      }
      double candidate = 0.0;
      int candidate_parent = current;
      const int from = parent[current];
      if (params.algorithm == Algorithm::kThetaStar && from != -1 &&
        line_of_sight(grid, from % width, from / width, nx, ny, params))
      {
        candidate = best[from] + line_cost(grid, from % width, from / width, nx, ny, params);
        candidate_parent = from;
      } else {
        candidate = best[current] + step_cost(grid, cx, cy, nx, ny, params);
      }
      if (candidate + kTiny < best[neighbour]) {
        best[neighbour] = candidate;
        parent[neighbour] = candidate_parent;
        open.emplace(candidate + heuristic(nx, ny), neighbour);
      }
    }
  }
  result.status = SearchStatus::kNoPath;
  return result;
}

std::vector<Point> smooth(
  const GridView & grid, const std::vector<Cell> & path, const SearchParams & params,
  double tolerance)
{
  std::vector<Point> kept;
  if (path.empty()) {
    return kept;
  }
  const std::size_t count = path.size();
  std::vector<double> cumulative(count, 0.0);
  for (std::size_t k = 1; k < count; ++k) {
    const Cell & a = path[k - 1];
    const Cell & b = path[k];
    const double segment = adjacent(a.first, a.second, b.first, b.second) ?
      step_cost(grid, a.first, a.second, b.first, b.second, params) :
      line_cost(grid, a.first, a.second, b.first, b.second, params);
    cumulative[k] = cumulative[k - 1] + segment;
  }
  kept.emplace_back(path.front().first, path.front().second);
  std::size_t i = 0;
  while (i + 1 < count) {
    std::size_t chosen = i + 1;
    for (std::size_t j = count - 1; j > i + 1; --j) {
      const double original = cumulative[j] - cumulative[i];
      const double shortcut = line_cost(
        grid, path[i].first, path[i].second, path[j].first, path[j].second, params);
      if (shortcut <= (1.0 + tolerance) * original + kTiny &&
        line_of_sight(grid, path[i].first, path[i].second, path[j].first, path[j].second, params))
      {
        chosen = j;
        break;
      }
    }
    kept.emplace_back(path[chosen].first, path[chosen].second);
    i = chosen;
  }
  return kept;
}

std::vector<Point> resample(const std::vector<Point> & points, double spacing)
{
  if (points.size() < 2 || spacing <= 0.0) {
    return points;
  }
  std::vector<Point> out = {points.front()};
  for (std::size_t k = 1; k < points.size(); ++k) {
    const Point & p = points[k - 1];
    const Point & q = points[k];
    const double length = std::hypot(q.first - p.first, q.second - p.second);
    if (length < kTiny) {
      continue;
    }
    const int steps = static_cast<int>(std::ceil(length / spacing - kTiny));
    for (int s = 1; s < steps; ++s) {
      const double t = static_cast<double>(s) / steps;
      out.emplace_back(p.first + t * (q.first - p.first), p.second + t * (q.second - p.second));
    }
    out.push_back(q);
  }
  return out;
}

}  // namespace rapid_nav_planner
