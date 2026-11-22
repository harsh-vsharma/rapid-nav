// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).
//
// A Nav2 costmap layer that paints where tracked obstacles will be over a short horizon. It reads
// a rapid_nav_msgs/TrackedObstacleArray (from the oracle, or any other velocity source) and writes
// the predicted disks into the master grid, after the inflation layer.

#ifndef RAPID_NAV_HAZARD__HAZARD_LAYER_HPP_
#define RAPID_NAV_HAZARD__HAZARD_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "rapid_nav_hazard/hazard_core.hpp"
#include "rapid_nav_msgs/msg/tracked_obstacle_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rapid_nav_hazard
{

class HazardLayer : public nav2_costmap_2d::Layer
{
public:
  HazardLayer() = default;
  ~HazardLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  void on_message(rapid_nav_msgs::msg::TrackedObstacleArray::ConstSharedPtr msg);

  rclcpp::Subscription<rapid_nav_msgs::msg::TrackedObstacleArray>::SharedPtr sub_;
  std::mutex mutex_;
  rapid_nav_msgs::msg::TrackedObstacleArray::ConstSharedPtr latest_;
  HazardParams params_;
  std::vector<Disk> disks_;  // this cycle's prediction, in the costmap's global frame
  BoundingBox last_box_;     // what the last cycle painted
};

}  // namespace rapid_nav_hazard

#endif  // RAPID_NAV_HAZARD__HAZARD_LAYER_HPP_
