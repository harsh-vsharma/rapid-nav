# Global planning and localisation

This page describes the global side of the navigation stack: the occupancy maps, localisation on the map, the global
costmap, the planner server with our planner plugin, and the tools that check them. The local controller (the
costmap around the robot and the velocity commands) arrives in a later stage.

## Maps

Two maps are committed in `src/rapid_nav_navigation/maps/`:

| map | size | origin (m) | what it is |
|---|---|---|---|
| `arena` | 220 × 180 cells | (-5.5, -4.5) | the arena, rasterised from the collision shapes of its walls, crates and pillars |
| `maze` | 200 × 160 cells | (-5.0, -4.0) | a fixed maze: an outer wall, a divider with a 0.70 m and a 0.45 m door, a U-shaped trap and a dead-end corridor |

Both are 0.05 m per cell, trinary PGM with YAML files that `map_server` reads. The arena map includes the free space
outside the walls; the planner's queries never start or end there.

To regenerate them (the output is deterministic, and `test_maps` checks the committed files):

```bash
uv run --with numpy --with pillow python src/rapid_nav_navigation/scripts/make_maps.py
```

The arena keeps the shapes whose height overlaps the robot's height band (0.02 m to 0.40 m), which drops the floor.
The maze is a list of rectangles in `make_maps.py`; the module docstring draws it.

## Localisation

`planning.launch.py` starts AMCL (`nav2_amcl`) on the arena map with the differential motion model and the likelihood
field for the laser (`/scan`, 60 beams, 500 to 2000 particles). AMCL starts at the robot's spawn pose and publishes
the `map → odom` transform; `odom → base_footprint` stays with the odometry of S1.

Two modes:

- `localization:=amcl` (default): the simulation. `sim.launch.py planning:=true` starts it with the spawn pose.
- `localization:=static`: no simulation. `map → odom` is the identity and `odom → base_footprint` is the initial pose,
  so the planner can be exercised without Gazebo.

## Costmap

The global costmap is built from the static map only (no obstacle layer): its resolution is 0.05 m, its frame is
`map`, and it follows `base_footprint`. The inflation layer uses the robot radius of 0.25 m as the inscribed radius,
an inflation radius of 0.55 m and a cost scaling factor of 3.0. Cells are lethal when occupied, inscribed within the
robot radius, and decay outside it.

## The planners

The planner server runs seven planners, all of them selectable by `planner_id`:

| `planner_id` | implementation | what it is |
|---|---|---|
| `RapidAStar` | ours | A* on the cost grid, 8-connected, no corner cutting |
| `RapidAStarSmooth` | ours | A*, then greedy shortcutting of the path within 10 % of its cost |
| `RapidDijkstra` | ours | Dijkstra on the same grid: the reference for `RapidAStar` |
| `RapidThetaStar` | ours | Theta*: any-angle paths where the straight line between two cells is free |
| `NavFn` | `nav2_navfn_planner` | potential field (stock Nav2) |
| `Smac2D` | `nav2_smac_planner` | 2-D A* with Nav2's cost penalties (stock Nav2) |
| `NavThetaStar` | `nav2_theta_star_planner` | Theta* (stock Nav2) |

Our four planners are one plugin, `rapid_nav_planner/GridPlanner` (`grid_planner_plugin.xml`), created four times with
different `algorithm` parameters.

### How our search works

- **Dijkstra** expands the cheapest cell first. It is optimal and uses no heuristic.
- **A*** is the same search with the octile distance to the goal as the heuristic. Each step costs at least its
  length, so the heuristic never overestimates: A* finds the same optimal cost as Dijkstra with fewer expansions.
- **Theta*** links a neighbour directly to the parent of the cell being expanded when the straight line between them
  is free (checked cell by cell, the supercover of the line). The path is then a set of straight segments, and each
  segment is costed by the length it spends in each cell.
- **Smoothing** (`RapidAStarSmooth`) walks the A* path and joins two points when the straight line between them is
  free and costs at most 10 % more than the original piece.
- **Cost** of entering a cell: its length (1 or √2 cells) times `1 + cost_weight × cost / 252`. Unknown cells count as
  cost 0 and are allowed. Lethal (254) and inscribed (253) cells are blocked.
- A start inside the inscribed zone is allowed (it is treated as free so that the robot can leave); a start on a
  lethal cell or a goal in an obstacle is an error, with the Nav2 codes `START_OCCUPIED` and `GOAL_OCCUPIED`.

The path returned to Nav2 is resampled every 0.05 m, and the search statistics (algorithm, expansions, cost, cells,
time, status) are published as JSON on `/<planner_id>/stats`.

### Parameters

| parameter | default | meaning |
|---|---|---|
| `algorithm` | `astar` | `astar`, `dijkstra` or `theta_star` |
| `cost_weight` | 2.0 | how strongly the inflation cost counts against length |
| `heuristic_weight` | 1.0 | weight of the A* heuristic (1.0 is optimal) |
| `lethal_cost` | 253 | cells at or above this cost are blocked |
| `allow_unknown` | true | whether unknown cells can be searched |
| `smooth` | false | smoothing of the path (`RapidAStarSmooth` sets it) |
| `smooth_tolerance` | 0.1 | allowed extra cost of a shortcut, as a fraction |
| `path_resolution` | 0.05 | spacing of the returned poses (m) |
| `max_planning_time` | 2.0 | the search gives up after this many seconds |

The values are in `src/rapid_nav_navigation/config/planning.yaml`.

## Tools

- **`path_viewer`** answers each goal from RViz ("2D Goal Pose") with every planner, from the robot's current pose. Each
  path is published on `/planned_paths/<planner_id>`, one colour per planner in `navigation.rviz`. The viewer asks one
  planner at a time, because the planner server runs one goal at a time. It runs when `planning.launch.py` starts with
  AMCL and `path_viewer:=true`.
- **`planner_check`** asks every planner every query in `config/planner_queries.yaml` and writes a bundle: `summary.md`,
  `table_<map>.md`, `results.csv`, `results.json`, `paths_<map>.png`. It checks feasible queries (every planner
  succeeds), infeasible ones (the right error code, with no path), the clearance of every path (at least 0.20 m from
  the nearest occupied cell centre), that `RapidAStar` is within 2 % of `RapidDijkstra`, that `RapidThetaStar` is at
  most 0.05 m longer than `RapidAStar`, and the planning time of our planners. In `--mode sim` it first checks AMCL
  against the ground truth through the drive of the S1 check.

  ```bash
  ros2 run rapid_nav_tools planner_check --map arena --mode offline --out planner_check_arena
  ```

- **`planner_bench`** times the search on random queries (no ROS): Dijkstra, A*, A* with heuristic weight 1.5 and
  Theta* on the cost grid of a map, and prints one CSV line per search.

  ```bash
  ros2 run rapid_nav_planner planner_bench --map <map yaml> --queries 200 --seed 1
  ```

  Queries are drawn from the largest region of free cells, which the search can reach from one another. The free
  space outside the arena walls is a separate region and gives no queries.

## Measured results

The numbers below were measured in the Docker development image on an arm64 virtual machine (8 CPUs, 4.8 GiB), which
is not a native machine: times are indicative only. The tester's native numbers come from the S5 checks on a native
install.

Offline `planner_check`, feasible queries (9 on the arena, 7 on the maze):

| map | planner | success | mean length (m) | mean min clearance (m) | mean heading change (deg/m) | median round trip (ms) |
|---|---|---|---|---|---|---|
| arena | RapidAStar | 9/9 | 4.05 | 0.789 | 91.3 | 2.7 |
| arena | RapidAStarSmooth | 9/9 | 3.85 | 0.773 | 16.2 | 2.5 |
| arena | RapidDijkstra | 9/9 | 4.05 | 0.789 | 52.4 | 6.4 |
| arena | RapidThetaStar | 9/9 | 3.77 | 0.655 | 10.8 | 34.5 |
| arena | NavFn | 9/9 | 3.97 | 0.811 | 125.9 | 2.5 |
| arena | Smac2D | 9/9 | 3.88 | 0.784 | 27.6 | 2.1 |
| arena | NavThetaStar | 9/9 | 3.86 | 0.790 | 29.3 | 1.7 |
| maze | RapidAStar | 7/7 | 6.02 | 0.925 | 68.1 | 7.5 |
| maze | RapidAStarSmooth | 7/7 | 5.60 | 0.892 | 10.9 | 6.0 |
| maze | RapidDijkstra | 7/7 | 6.02 | 0.925 | 43.8 | 12.8 |
| maze | RapidThetaStar | 7/7 | 5.50 | 0.863 | 10.7 | 104.1 |
| maze | NavFn | 7/7 | 5.83 | 0.917 | 96.3 | 3.8 |
| maze | Smac2D | 7/7 | 5.76 | 0.918 | 24.9 | 4.5 |
| maze | NavThetaStar | 7/7 | 5.64 | 0.918 | 20.4 | 2.9 |

Two stock Nav2 planners do not give the expected answer on the infeasible queries, and `planner_check` reports it:
NavFn returns a path into a crate when the goal lies inside it, and Smac2D returns `NO_VALID_PATH` where the goal lies
in an obstacle instead of `GOAL_OCCUPIED`.

`planner_bench`, 200 random queries with seed 1 (the same queries for every algorithm):

| map | algorithm | found | mean cost | mean length (m) | mean expansions | median time (ms) |
|---|---|---|---|---|---|---|
| arena | Dijkstra | 200/200 | 93.62 | 4.59 | 12454 | 9.05 |
| arena | A* | 200/200 | 93.62 | 4.59 | 878 | 0.49 |
| arena | A* × 1.5 | 200/200 | 94.89 | 4.65 | 127 | 0.32 |
| arena | Theta* | 200/200 | 105.51 | 4.32 | 2656 | 16.24 |
| maze | Dijkstra | 200/200 | 138.46 | 5.99 | 12565 | 9.57 |
| maze | A* | 200/200 | 138.46 | 5.99 | 4090 | 2.93 |
| maze | A* × 1.5 | 200/200 | 140.64 | 6.03 | 2239 | 1.76 |
| maze | Theta* | 200/200 | 157.94 | 5.48 | 5232 | 48.45 |

Dijkstra and A* found the same cost on every query. Theta* gives shorter paths but a higher weighted cost: it trades
the inflation cost for length.
