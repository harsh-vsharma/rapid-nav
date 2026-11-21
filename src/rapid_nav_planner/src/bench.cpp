// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).
// Benchmark of the grid search (see bench.hpp).

#include "rapid_nav_planner/bench.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <ostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rapid_nav_planner
{
namespace
{

constexpr std::uint8_t kCostInscribed = 253;
constexpr double kOuterRadius = 0.55;
constexpr double kCostScalingFactor = 3.0;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInfinity = std::numeric_limits<double>::infinity();

const char * status_name(SearchStatus status)
{
  switch (status) {
    case SearchStatus::kFound:
      return "found";
    case SearchStatus::kStartBlocked:
      return "start_blocked";
    case SearchStatus::kGoalBlocked:
      return "goal_blocked";
    case SearchStatus::kNoPath:
      return "no_path";
    case SearchStatus::kCancelled:
      return "cancelled";
    case SearchStatus::kTimedOut:
      return "timed_out";
  }
  return "unknown";
}

std::string trim(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t\r");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = text.find_last_not_of(" \t\r");
  return text.substr(first, last - first + 1);
}

// The numbers in "[a, b, c]" or "a b", in order.
std::vector<double> numbers_in(const std::string & text)
{
  std::string cleaned = text;
  for (char & c : cleaned) {
    if (c == '[' || c == ']' || c == ',') {
      c = ' ';
    }
  }
  std::istringstream in(cleaned);
  std::vector<double> values;
  double value = 0.0;
  while (in >> value) {
    values.push_back(value);
  }
  return values;
}

// The fields of a map YAML that the benchmark needs (map_server's names).
struct MapFile
{
  std::string image;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double occupied_thresh = 0.65;
  double free_thresh = 0.25;
  bool negate = false;
};

MapFile read_map_yaml(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot open " + path);
  }
  std::map<std::string, std::string> fields;
  std::string line;
  while (std::getline(in, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) {
      line.erase(hash);
    }
    const auto colon = line.find(':');
    if (colon != std::string::npos) {
      fields[trim(line.substr(0, colon))] = trim(line.substr(colon + 1));
    }
  }
  MapFile map;
  map.image = fields.at("image");
  map.resolution = std::stod(fields.at("resolution"));
  const std::vector<double> origin = numbers_in(fields.at("origin"));
  if (origin.size() < 2) {
    throw std::runtime_error("origin needs x and y in " + path);
  }
  map.origin_x = origin[0];
  map.origin_y = origin[1];
  if (fields.count("occupied_thresh") > 0) {
    map.occupied_thresh = std::stod(fields["occupied_thresh"]);
  }
  if (fields.count("free_thresh") > 0) {
    map.free_thresh = std::stod(fields["free_thresh"]);
  }
  map.negate = fields.count("negate") > 0 && fields["negate"] == "1";
  return map;
}

struct PgmImage
{
  int width = 0;
  int height = 0;
  std::vector<unsigned char> pixels;  // rows from the top of the image
};

// Read a binary PGM (P5, maxval 255).
PgmImage read_pgm(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("cannot open " + path);
  }
  std::string magic;
  in >> magic;
  if (magic != "P5") {
    throw std::runtime_error("not a binary PGM: " + path);
  }
  const auto next_token = [&in]() {
      std::string token;
      while (true) {
        in >> std::ws;
        if (in.peek() != '#') {
          break;
        }
        std::string comment;
        std::getline(in, comment);
      }
      in >> token;
      return token;
    };
  PgmImage image;
  image.width = std::stoi(next_token());
  image.height = std::stoi(next_token());
  const int maxval = std::stoi(next_token());
  if (maxval != 255) {
    throw std::runtime_error("maxval must be 255 in " + path);
  }
  in.get();  // the single whitespace byte before the raster
  image.pixels.resize(static_cast<std::size_t>(image.width) * image.height);
  in.read(reinterpret_cast<char *>(image.pixels.data()),
    static_cast<std::streamsize>(image.pixels.size()));
  if (!in) {
    throw std::runtime_error("the raster of " + path + " is short");
  }
  return image;
}

// The cells of the largest region of free cells, found with the search's neighbour rule: eight
// neighbours, and a diagonal step only when both cells beside it are free.
std::vector<char> largest_region(const BenchGrid & grid)
{
  const int width = grid.width;
  const int height = grid.height;
  const auto is_free = [&grid, width, height](int x, int y) {
      return x >= 0 && y >= 0 && x < width && y < height &&
             grid.costs[static_cast<std::size_t>(y) * width + x] < kCostInscribed;
    };
  const std::size_t count = static_cast<std::size_t>(width) * height;
  std::vector<int> label(count, -1);
  std::vector<std::size_t> sizes;
  for (int start = 0; start < static_cast<int>(count); ++start) {
    const int sx = start % width;
    const int sy = start / width;
    if (label[start] != -1 || !is_free(sx, sy)) {
      continue;
    }
    const int id = static_cast<int>(sizes.size());
    sizes.push_back(0);
    label[start] = id;
    std::vector<std::pair<int, int>> stack{{sx, sy}};
    while (!stack.empty()) {
      const auto [cx, cy] = stack.back();
      stack.pop_back();
      ++sizes[id];
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int nx = cx + dx;
          const int ny = cy + dy;
          if ((dx == 0 && dy == 0) || !is_free(nx, ny)) {
            continue;
          }
          if (dx != 0 && dy != 0 && (!is_free(cx + dx, cy) || !is_free(cx, cy + dy))) {
            continue;
          }
          const int neighbour = ny * width + nx;
          if (label[neighbour] == -1) {
            label[neighbour] = id;
            stack.emplace_back(nx, ny);
          }
        }
      }
    }
  }
  std::vector<char> mask(count, 0);
  if (sizes.empty()) {
    return mask;
  }
  const int largest = static_cast<int>(std::distance(
    sizes.begin(), std::max_element(sizes.begin(), sizes.end())));
  for (std::size_t i = 0; i < count; ++i) {
    mask[i] = label[i] == largest ? 1 : 0;
  }
  return mask;
}

double cells_length(const std::vector<Cell> & cells, double resolution)
{
  double length = 0.0;
  for (std::size_t k = 1; k < cells.size(); ++k) {
    const double dx = cells[k].first - cells[k - 1].first;
    const double dy = cells[k].second - cells[k - 1].second;
    length += std::hypot(dx, dy);
  }
  return length * resolution;
}

}  // namespace

BenchGrid build_cost_grid(
  const std::vector<std::uint8_t> & occupied, int width, int height, double resolution,
  double origin_x, double origin_y, double inflate_radius)
{
  const std::size_t count = static_cast<std::size_t>(width) * height;
  if (occupied.size() != count) {
    throw std::invalid_argument("the occupancy mask does not match the grid size");
  }
  BenchGrid grid;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  grid.origin_x = origin_x;
  grid.origin_y = origin_y;

  // Distance in metres from every cell to the nearest occupied cell centre, up to the outer radius.
  std::vector<double> distance(count, kInfinity);
  const int reach = static_cast<int>(std::ceil(kOuterRadius / resolution));
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (occupied[static_cast<std::size_t>(y) * width + x] == 0) {
        continue;
      }
      for (int dy = -reach; dy <= reach; ++dy) {
        for (int dx = -reach; dx <= reach; ++dx) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
            continue;
          }
          const double d = std::hypot(dx, dy) * resolution;
          if (d > kOuterRadius) {
            continue;
          }
          double & slot = distance[static_cast<std::size_t>(ny) * width + nx];
          slot = std::min(slot, d);
        }
      }
    }
  }

  grid.costs.assign(count, 0);
  for (std::size_t i = 0; i < count; ++i) {
    if (occupied[i] != 0) {
      grid.costs[i] = kCostLethal;
    } else if (distance[i] <= inflate_radius) {
      grid.costs[i] = kCostInscribed;
    } else if (distance[i] <= kOuterRadius) {
      const double cost = 252.0 * std::exp(-kCostScalingFactor * (distance[i] - inflate_radius));
      grid.costs[i] = static_cast<std::uint8_t>(std::clamp(std::lround(cost), 1L, 252L));
    }
  }
  return grid;
}

std::string load_grid(const std::string & map_yaml, double inflate_radius, BenchGrid & grid)
{
  try {
    const MapFile map = read_map_yaml(map_yaml);
    const std::filesystem::path image_path =
      std::filesystem::path(map_yaml).parent_path() / map.image;
    const PgmImage image = read_pgm(image_path.string());
    // The PGM stores the top row first; the grid's row 0 is the bottom of the map.
    std::vector<std::uint8_t> occupied(image.pixels.size(), 0);
    for (int row = 0; row < image.height; ++row) {
      for (int column = 0; column < image.width; ++column) {
        const double value = image.pixels[static_cast<std::size_t>(row) * image.width + column];
        const double occupancy = map.negate ? value / 255.0 : (255.0 - value) / 255.0;
        const int grid_row = image.height - 1 - row;
        occupied[static_cast<std::size_t>(grid_row) * image.width + column] =
          occupancy > map.occupied_thresh ? 1 : 0;
      }
    }
    grid = build_cost_grid(occupied, image.width, image.height, map.resolution, map.origin_x,
        map.origin_y, inflate_radius);
  } catch (const std::exception & error) {
    return error.what();
  }
  return "";
}

std::vector<BenchQuery> random_queries(const BenchGrid & grid, int count, unsigned int seed)
{
  const std::vector<char> region = largest_region(grid);
  std::vector<Cell> free_cells;
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      if (region[static_cast<std::size_t>(y) * grid.width + x] != 0) {
        free_cells.emplace_back(x, y);
      }
    }
  }
  std::vector<BenchQuery> queries;
  if (free_cells.size() < 2) {
    return queries;
  }
  std::mt19937 generator(seed);
  std::uniform_int_distribution<std::size_t> pick(0, free_cells.size() - 1);
  for (int k = 0; k < count; ++k) {
    const Cell start = free_cells[pick(generator)];
    Cell goal = start;
    while (goal == start) {
      goal = free_cells[pick(generator)];
    }
    std::ostringstream name;
    name << 'q' << std::setw(4) << std::setfill('0') << (k + 1);
    queries.push_back(BenchQuery{name.str(), start, goal});
  }
  return queries;
}

std::vector<BenchAlgorithm> default_algorithms(double cost_weight)
{
  SearchParams base;
  base.cost_weight = cost_weight;
  base.lethal_cost = kCostInscribed;
  base.allow_unknown = true;

  SearchParams dijkstra = base;
  dijkstra.algorithm = Algorithm::kDijkstra;
  SearchParams astar = base;
  astar.algorithm = Algorithm::kAStar;
  astar.heuristic_weight = 1.0;
  SearchParams weighted = astar;
  weighted.heuristic_weight = 1.5;
  SearchParams theta = base;
  theta.algorithm = Algorithm::kThetaStar;

  return {
    BenchAlgorithm{"dijkstra", dijkstra},
    BenchAlgorithm{"astar", astar},
    BenchAlgorithm{"astar_w1.5", weighted},
    BenchAlgorithm{"theta_star", theta},
  };
}

std::vector<BenchRow> run_bench(
  const BenchGrid & grid, const std::vector<BenchQuery> & queries,
  const std::vector<BenchAlgorithm> & algorithms)
{
  std::vector<BenchRow> rows;
  const GridView view(grid.width, grid.height, grid.costs.data());
  const std::function<bool()> no_cancel;
  for (const BenchQuery & query : queries) {
    for (const BenchAlgorithm & algorithm : algorithms) {
      const auto started = std::chrono::steady_clock::now();
      const SearchResult result =
        search(view, query.start, query.goal, algorithm.params, no_cancel);
      const auto finished = std::chrono::steady_clock::now();

      BenchRow row;
      row.query = query.name;
      row.algorithm = algorithm.name;
      row.status = status_name(result.status);
      row.expansions = result.expansions;
      row.time_us = std::chrono::duration_cast<std::chrono::microseconds>(
        finished - started).count();
      if (result.status == SearchStatus::kFound) {
        row.cost = result.cost;
        row.length_m = cells_length(result.cells, grid.resolution);
      } else {
        row.cost = kNaN;
        row.length_m = kNaN;
      }
      rows.push_back(row);
    }
  }
  return rows;
}

void write_csv(std::ostream & out, const std::vector<BenchRow> & rows)
{
  out << "query,algorithm,status,cost,length_m,expansions,time_us\n";
  out << std::fixed << std::setprecision(3);
  for (const BenchRow & row : rows) {
    out << row.query << ',' << row.algorithm << ',' << row.status << ',' << row.cost << ','
        << row.length_m << ',' << row.expansions << ',' << row.time_us << '\n';
  }
}

}  // namespace rapid_nav_planner
