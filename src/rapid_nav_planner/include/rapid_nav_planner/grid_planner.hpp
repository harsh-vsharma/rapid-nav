// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).

// Nav2 global planner plugin around the grid search library: Dijkstra, A* or Theta* on the
// global costmap, with optional smoothing and resampling. Each call publishes its statistics
// on <node>/<name>/stats.

#ifndef RAPID_NAV_PLANNER__GRID_PLANNER_HPP_
#define RAPID_NAV_PLANNER__GRID_PLANNER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rapid_nav_planner/compat.hpp"
#include "rapid_nav_planner/grid_search.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.hpp"

namespace rapid_nav_planner
{

class GridPlanner : public nav2_core::GlobalPlanner
{
public:
  GridPlanner() = default;
  ~GridPlanner() override = default;

  void configure(
    const compat::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
#if RAPID_NAV_NAV2_ROS_COMMON
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
#endif
    std::function<bool()> cancel_checker) override;

private:
  void publish_stats(
    const std::string & status, std::size_t expansions, double cost, std::size_t cells,
    double time_ms);

  compat::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  std::string algorithm_name_;
  std::string global_frame_;
  SearchParams params_;
  bool smooth_ = false;
  double smooth_tolerance_ = 0.1;
  double path_resolution_ = 0.05;
  double max_planning_time_ = 2.0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
};

}  // namespace rapid_nav_planner

#endif  // RAPID_NAV_PLANNER__GRID_PLANNER_HPP_
