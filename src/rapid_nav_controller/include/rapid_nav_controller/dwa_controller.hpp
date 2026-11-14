// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).
//
// Our dynamic window approach as a Nav2 controller plugin. The plugin takes the global plan into
// the local costmap's frame, prunes it, copies the costmap, calls dwa_core and publishes the chosen
// trajectory (and, when asked, every candidate).

#ifndef RAPID_NAV_CONTROLLER__DWA_CONTROLLER_HPP_
#define RAPID_NAV_CONTROLLER__DWA_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rapid_nav_controller/dwa_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rapid_nav_controller
{

class DwaController : public nav2_core::Controller
{
public:
  DwaController();
  ~DwaController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void load_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  double read_double(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & key,
    double value) const;
  int read_int(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & key,
    int value) const;
  bool read_bool(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & key,
    bool value) const;

  // The global plan in the costmap's frame, one pose per point; false when a transform fails.
  bool transform_plan(std::vector<Pose2D> & plan) const;
  CostGrid copy_costmap() const;
  void publish_local_plan(const Trajectory & best, const std::string & frame);
  void publish_trajectories(const DwaResult & result, const std::string & frame);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  DwaParams params_;
  double transform_tolerance_ = 0.2;
  bool publish_trajectories_ = false;
  double speed_limit_;  // the forward speed cap set by setSpeedLimit (infinite when none)

  nav_msgs::msg::Path global_plan_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    trajectories_pub_;
};

}  // namespace rapid_nav_controller

#endif  // RAPID_NAV_CONTROLLER__DWA_CONTROLLER_HPP_
