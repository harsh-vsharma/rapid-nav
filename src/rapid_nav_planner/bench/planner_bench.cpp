// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).
// planner_bench: time Dijkstra, A*, weighted A* and Theta* on random queries of a map; prints CSV.

#include <exception>
#include <iostream>
#include <string>

#include "rapid_nav_planner/bench.hpp"

namespace
{

struct Options
{
  std::string map;
  int queries = 200;
  unsigned int seed = 1;
  double cost_weight = 2.0;
  double inflate = 0.25;
};

void usage(std::ostream & out)
{
  out << "usage: planner_bench --map <yaml> [--queries N] [--seed S]\n"
      << "                     [--cost-weight W] [--inflate R]\n"
      << "  --map          the map's YAML (trinary, as written by make_maps.py)\n"
      << "  --queries      random start and goal pairs (default 200)\n"
      << "  --seed         seed of the queries (default 1)\n"
      << "  --cost-weight  the planner's cost weight (default 2.0)\n"
      << "  --inflate      inscribed radius in metres: cells within it cost 253 (default 0.25)\n";
}

// Every option takes one value. Returns false for an unknown option or a missing value.
bool parse(int argc, char ** argv, Options & options)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (i + 1 >= argc) {
      return false;
    }
    const std::string value = argv[++i];
    if (arg == "--map") {
      options.map = value;
    } else if (arg == "--queries") {
      options.queries = std::stoi(value);
    } else if (arg == "--seed") {
      options.seed = static_cast<unsigned int>(std::stoul(value));
    } else if (arg == "--cost-weight") {
      options.cost_weight = std::stod(value);
    } else if (arg == "--inflate") {
      options.inflate = std::stod(value);
    } else {
      return false;
    }
  }
  return !options.map.empty() && options.queries > 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  try {
    if (!parse(argc, argv, options)) {
      usage(std::cerr);
      return 2;
    }
  } catch (const std::exception & error) {
    std::cerr << "planner_bench: " << error.what() << '\n';
    usage(std::cerr);
    return 2;
  }

  rapid_nav_planner::BenchGrid grid;
  const std::string error = rapid_nav_planner::load_grid(options.map, options.inflate, grid);
  if (!error.empty()) {
    std::cerr << "planner_bench: " << error << '\n';
    return 1;
  }
  const auto queries = rapid_nav_planner::random_queries(grid, options.queries, options.seed);
  const auto algorithms = rapid_nav_planner::default_algorithms(options.cost_weight);
  const auto rows = rapid_nav_planner::run_bench(grid, queries, algorithms);
  rapid_nav_planner::write_csv(std::cout, rows);
  return 0;
}
