// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).

#include "rapid_nav_planner/grid_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rapid_nav_planner
{
namespace
{

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

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quaternion_of(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

}  // namespace

void GridPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("GridPlanner: the parent node is gone");
  }
  name_ = name;
  costmap_ros_ = costmap_ros;
  global_frame_ = costmap_ros_->getGlobalFrameID();

  const auto declare = [this](const std::string & key, const rclcpp::ParameterValue & value) {
      nav2_util::declare_parameter_if_not_declared(node_, name_ + "." + key, value);
    };
  declare("algorithm", rclcpp::ParameterValue(std::string("astar")));
  declare("cost_weight", rclcpp::ParameterValue(2.0));
  declare("heuristic_weight", rclcpp::ParameterValue(1.0));
  declare("lethal_cost", rclcpp::ParameterValue(253));
  declare("allow_unknown", rclcpp::ParameterValue(true));
  declare("smooth", rclcpp::ParameterValue(false));
  declare("smooth_tolerance", rclcpp::ParameterValue(0.1));
  declare("path_resolution", rclcpp::ParameterValue(0.05));
  declare("max_planning_time", rclcpp::ParameterValue(2.0));

  algorithm_name_ = node_->get_parameter(name_ + ".algorithm").as_string();
  if (algorithm_name_ == "astar") {
    params_.algorithm = Algorithm::kAStar;
  } else if (algorithm_name_ == "dijkstra") {
    params_.algorithm = Algorithm::kDijkstra;
  } else if (algorithm_name_ == "theta_star") {
    params_.algorithm = Algorithm::kThetaStar;
  } else {
    throw std::runtime_error(
            "GridPlanner: algorithm must be astar, dijkstra or theta_star, got '" +
            algorithm_name_ + "'");
  }
  params_.cost_weight = node_->get_parameter(name_ + ".cost_weight").as_double();
  params_.heuristic_weight = node_->get_parameter(name_ + ".heuristic_weight").as_double();
  params_.lethal_cost = static_cast<std::uint8_t>(
    node_->get_parameter(name_ + ".lethal_cost").as_int());
  params_.allow_unknown = node_->get_parameter(name_ + ".allow_unknown").as_bool();
  smooth_ = node_->get_parameter(name_ + ".smooth").as_bool();
  smooth_tolerance_ = node_->get_parameter(name_ + ".smooth_tolerance").as_double();
  path_resolution_ = node_->get_parameter(name_ + ".path_resolution").as_double();
  max_planning_time_ = node_->get_parameter(name_ + ".max_planning_time").as_double();

  stats_pub_ = node_->create_publisher<std_msgs::msg::String>(name_ + "/stats", rclcpp::QoS(1));
  RCLCPP_INFO(
    node_->get_logger(), "GridPlanner '%s': algorithm=%s cost_weight=%.2f smooth=%s",
    name_.c_str(), algorithm_name_.c_str(), params_.cost_weight, smooth_ ? "true" : "false");
}

void GridPlanner::cleanup()
{
  stats_pub_.reset();
  costmap_ros_.reset();
  node_.reset();
}

void GridPlanner::activate()
{
}

void GridPlanner::deactivate()
{
}

void GridPlanner::publish_stats(
  const std::string & status, std::size_t expansions, double cost, std::size_t cells,
  double time_ms)
{
  if (!stats_pub_) {
    return;
  }
  std::ostringstream json;
  json << std::fixed << std::setprecision(3);
  json << "{\"algorithm\": \"" << algorithm_name_ << "\", \"expansions\": " << expansions
       << ", \"cost\": " << cost << ", \"cells\": " << cells << ", \"time_ms\": " << time_ms
       << ", \"status\": \"" << status << "\"}";
  std_msgs::msg::String message;
  message.data = json.str();
  stats_pub_->publish(message);
}

nav_msgs::msg::Path GridPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  const auto started = std::chrono::steady_clock::now();
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  const unsigned int size_x = costmap->getSizeInCellsX();
  const unsigned int size_y = costmap->getSizeInCellsY();

  unsigned int start_x = 0;
  unsigned int start_y = 0;
  unsigned int goal_x = 0;
  unsigned int goal_y = 0;
  if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
    throw nav2_core::StartOutsideMapBounds("GridPlanner: the start is outside the costmap");
  }
  if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
    throw nav2_core::GoalOutsideMapBounds("GridPlanner: the goal is outside the costmap");
  }

  // Copy the costs under the costmap's lock; the search runs on the copy.
  std::vector<std::uint8_t> costs(static_cast<std::size_t>(size_x) * size_y);
  {
    std::lock_guard<std::recursive_mutex> lock(*costmap->getMutex());
    std::copy(costmap->getCharMap(), costmap->getCharMap() + costs.size(), costs.begin());
  }
  const std::size_t start_index = static_cast<std::size_t>(start_y) * size_x + start_x;
  const std::uint8_t start_cost = costs[start_index];
  if (start_cost == kCostLethal ||
    (start_cost == kCostUnknown && !params_.allow_unknown))
  {
    throw nav2_core::StartOccupied("GridPlanner: the start is in an obstacle");
  }
  // A robot inside the inscribed zone must still be able to leave it.
  if (start_cost != kCostUnknown && start_cost >= params_.lethal_cost) {
    costs[start_index] = 0;
  }

  const GridView grid(static_cast<int>(size_x), static_cast<int>(size_y), costs.data());
  if (grid.blocked(static_cast<int>(goal_x), static_cast<int>(goal_y), params_)) {
    throw nav2_core::GoalOccupied("GridPlanner: the goal is in an obstacle");
  }

  const auto to_world = [costmap](const Cell & cell) {
      double wx = 0.0;
      double wy = 0.0;
      costmap->mapToWorld(static_cast<unsigned int>(cell.first),
        static_cast<unsigned int>(cell.second), wx, wy);
      const double half = 0.5 * costmap->getResolution();
      return Point(wx + half, wy + half);
    };

  std::vector<Point> points;
  std::size_t expansions = 0;
  double cost = 0.0;
  std::size_t cell_count = 0;
  if (start_x == goal_x && start_y == goal_y) {
    points = {Point(goal.pose.position.x, goal.pose.position.y)};
    cell_count = 1;
    publish_stats("found", 0, 0.0, cell_count, 0.0);
  } else {
    SearchParams params = params_;
    params.deadline = started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(max_planning_time_));
    const SearchResult result = search(
      grid, Cell(static_cast<int>(start_x), static_cast<int>(start_y)),
      Cell(static_cast<int>(goal_x), static_cast<int>(goal_y)), params, cancel_checker);
    const double time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    expansions = result.expansions;
    cost = result.cost;
    cell_count = result.cells.size();
    publish_stats(status_name(result.status), expansions, cost, cell_count, time_ms);
    switch (result.status) {
      case SearchStatus::kFound:
        break;
      case SearchStatus::kCancelled:
        throw nav2_core::PlannerCancelled("GridPlanner: planning was cancelled");
      case SearchStatus::kTimedOut:
        throw nav2_core::PlannerTimedOut(
                "GridPlanner: planning took longer than max_planning_time");
      case SearchStatus::kStartBlocked:
        throw nav2_core::StartOccupied("GridPlanner: the start is in an obstacle");
      case SearchStatus::kGoalBlocked:
        throw nav2_core::GoalOccupied("GridPlanner: the goal is in an obstacle");
      case SearchStatus::kNoPath:
        throw nav2_core::NoValidPathCouldBeFound("GridPlanner: no path to the goal");
    }
    if (smooth_) {
      const std::vector<Point> smoothed = smooth(grid, result.cells, params_, smooth_tolerance_);
      for (const Point & cell_point : smoothed) {
        points.push_back(to_world(Cell(static_cast<int>(cell_point.first),
          static_cast<int>(cell_point.second))));
      }
    } else {
      for (const Cell & cell : result.cells) {
        points.push_back(to_world(cell));
      }
    }
    // The exact start and goal replace the cell centres at the two ends.
    points.front() = Point(start.pose.position.x, start.pose.position.y);
    points.back() = Point(goal.pose.position.x, goal.pose.position.y);
    points = resample(points, path_resolution_);
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node_->now();
  const double goal_yaw = yaw_of(goal.pose.orientation);
  path.poses.reserve(points.size());
  for (std::size_t k = 0; k < points.size(); ++k) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = points[k].first;
    pose.pose.position.y = points[k].second;
    pose.pose.position.z = 0.0;
    double yaw = goal_yaw;
    if (k + 1 < points.size()) {
      yaw = std::atan2(
        points[k + 1].second - points[k].second, points[k + 1].first - points[k].first);
    }
    pose.pose.orientation = quaternion_of(yaw);
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace rapid_nav_planner

PLUGINLIB_EXPORT_CLASS(rapid_nav_planner::GridPlanner, nav2_core::GlobalPlanner)
