// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).

#include "rapid_nav_controller/dwa_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace rapid_nav_controller
{
namespace
{

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
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

DwaController::DwaController()
: logger_(rclcpp::get_logger("DwaController")),
  speed_limit_(std::numeric_limits<double>::infinity())
{
}

void DwaController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  const auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("DwaController: the parent node has expired");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);

  load_parameters(node);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  trajectories_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("trajectories", 1);
  RCLCPP_INFO(logger_, "DwaController '%s' configured", name_.c_str());
}

void DwaController::cleanup()
{
  local_plan_pub_.reset();
  trajectories_pub_.reset();
  global_plan_.poses.clear();
  RCLCPP_INFO(logger_, "DwaController '%s' cleaned up", name_.c_str());
}

void DwaController::activate()
{
  local_plan_pub_->on_activate();
  trajectories_pub_->on_activate();
  RCLCPP_INFO(logger_, "DwaController '%s' activated", name_.c_str());
}

void DwaController::deactivate()
{
  local_plan_pub_->on_deactivate();
  trajectories_pub_->on_deactivate();
  RCLCPP_INFO(logger_, "DwaController '%s' deactivated", name_.c_str());
}

void DwaController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw nav2_core::InvalidPath("DwaController: the global plan is empty");
  }
  global_plan_ = path;
}

void DwaController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  // A non-positive or non-finite limit lifts the cap.
  if (!std::isfinite(speed_limit) || speed_limit <= 0.0) {
    speed_limit_ = std::numeric_limits<double>::infinity();
    return;
  }
  speed_limit_ = percentage ? params_.max_vel_x * speed_limit / 100.0 : speed_limit;
}

geometry_msgs::msg::TwistStamped DwaController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("DwaController: no global plan has been set");
  }
  const std::string frame = costmap_ros_->getGlobalFrameID();

  std::vector<Pose2D> plan;
  if (!transform_plan(plan)) {
    throw nav2_core::ControllerTFError("DwaController: the global plan could not be transformed");
  }
  const Pose2D robot{pose.pose.position.x, pose.pose.position.y, yaw_of(pose.pose.orientation)};
  const Pose2D goal = plan.back();
  const CostGrid grid = copy_costmap();

  // Prune: from the point nearest the robot on, keep the points inside the local costmap.
  const std::size_t nearest = nearest_path_index(plan, robot);
  std::vector<Pose2D> ahead;
  for (std::size_t i = nearest; i < plan.size(); ++i) {
    if (grid.cost_at(plan[i].x, plan[i].y) < 0) {
      break;
    }
    ahead.push_back(plan[i]);
  }
  if (ahead.empty()) {
    ahead.push_back(plan[nearest]);
  }

  DwaParams params = params_;
  params.max_vel_x = std::min(params_.max_vel_x, speed_limit_);
  const Velocity current{velocity.linear.x, velocity.angular.z};
  const DwaResult result = compute(grid, robot, current, ahead, goal, params);
  if (result.status == DwaStatus::kNoValidTrajectory) {
    throw nav2_core::NoValidControl("DwaController: no collision-free trajectory");
  }

  publish_local_plan(result.best, frame);
  if (publish_trajectories_) {
    publish_trajectories(result, frame);
  }

  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = clock_->now();
  command.header.frame_id = costmap_ros_->getBaseFrameID();
  command.twist.linear.x = result.cmd.v;
  command.twist.angular.z = result.cmd.w;
  return command;
}

void DwaController::load_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  nav2_util::declare_parameter_if_not_declared(
    node, "controller_frequency", rclcpp::ParameterValue(20.0));
  const double frequency = node->get_parameter("controller_frequency").as_double();

  DwaParams & p = params_;
  p.dt = 1.0 / frequency;
  p.max_vel_x = read_double(node, "max_vel_x", p.max_vel_x);
  p.min_vel_x = read_double(node, "min_vel_x", p.min_vel_x);
  p.max_vel_theta = read_double(node, "max_vel_theta", p.max_vel_theta);
  p.acc_lim_x = read_double(node, "acc_lim_x", p.acc_lim_x);
  p.acc_lim_theta = read_double(node, "acc_lim_theta", p.acc_lim_theta);
  p.vx_samples = read_int(node, "vx_samples", p.vx_samples);
  p.wz_samples = read_int(node, "wz_samples", p.wz_samples);
  p.sim_time = read_double(node, "sim_time", p.sim_time);
  p.sim_step = read_double(node, "sim_step", p.sim_step);
  p.lookahead = read_double(node, "lookahead", p.lookahead);
  p.rotate_to_heading_angle =
    read_double(node, "rotate_to_heading_angle", p.rotate_to_heading_angle);
  p.rotate_gain = read_double(node, "rotate_gain", p.rotate_gain);
  p.xy_goal_tolerance = read_double(node, "xy_goal_tolerance", p.xy_goal_tolerance);
  p.yaw_goal_tolerance = read_double(node, "yaw_goal_tolerance", p.yaw_goal_tolerance);
  p.path_distance_weight = read_double(node, "path_distance_weight", p.path_distance_weight);
  p.goal_distance_weight = read_double(node, "goal_distance_weight", p.goal_distance_weight);
  p.heading_weight = read_double(node, "heading_weight", p.heading_weight);
  p.obstacle_weight = read_double(node, "obstacle_weight", p.obstacle_weight);
  p.speed_weight = read_double(node, "speed_weight", p.speed_weight);
  p.lethal_threshold = read_int(node, "lethal_threshold", p.lethal_threshold);
  transform_tolerance_ = read_double(node, "transform_tolerance", transform_tolerance_);
  publish_trajectories_ = read_bool(node, "publish_trajectories", publish_trajectories_);
}

double DwaController::read_double(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & key,
  double value) const
{
  const std::string full = name_ + "." + key;
  nav2_util::declare_parameter_if_not_declared(node, full, rclcpp::ParameterValue(value));
  return node->get_parameter(full).as_double();
}

int DwaController::read_int(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & key,
  int value) const
{
  const std::string full = name_ + "." + key;
  nav2_util::declare_parameter_if_not_declared(node, full, rclcpp::ParameterValue(value));
  return static_cast<int>(node->get_parameter(full).as_int());
}

bool DwaController::read_bool(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & key,
  bool value) const
{
  const std::string full = name_ + "." + key;
  nav2_util::declare_parameter_if_not_declared(node, full, rclcpp::ParameterValue(value));
  return node->get_parameter(full).as_bool();
}

bool DwaController::transform_plan(std::vector<Pose2D> & plan) const
{
  const std::string frame = costmap_ros_->getGlobalFrameID();
  plan.clear();
  plan.reserve(global_plan_.poses.size());
  for (const geometry_msgs::msg::PoseStamped & stamped : global_plan_.poses) {
    geometry_msgs::msg::PoseStamped transformed = stamped;
    if (stamped.header.frame_id != frame) {
      // The latest transform, not the one at the plan's stamp, which may be too old by now.
      geometry_msgs::msg::PoseStamped input = stamped;
      input.header.stamp = rclcpp::Time(0);
      if (!nav2_util::transformPoseInTargetFrame(
          input, transformed, *tf_, frame, transform_tolerance_))
      {
        return false;
      }
    }
    plan.push_back(
      Pose2D{transformed.pose.position.x, transformed.pose.position.y,
        yaw_of(transformed.pose.orientation)});
  }
  return true;
}

CostGrid DwaController::copy_costmap() const
{
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  const auto size_x = static_cast<std::size_t>(costmap->getSizeInCellsX());
  const auto size_y = static_cast<std::size_t>(costmap->getSizeInCellsY());
  std::vector<std::uint8_t> costs;
  {
    // The costmap is updated by its own thread; copy it under its mutex.
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    const unsigned char * charmap = costmap->getCharMap();
    costs.assign(charmap, charmap + size_x * size_y);
  }
  return CostGrid(
    costmap->getOriginX(), costmap->getOriginY(), costmap->getResolution(),
    static_cast<int>(size_x), static_cast<int>(size_y), std::move(costs));
}

void DwaController::publish_local_plan(const Trajectory & best, const std::string & frame)
{
  nav_msgs::msg::Path local;
  local.header.frame_id = frame;
  local.header.stamp = clock_->now();
  local.poses.reserve(best.poses.size());
  for (const Pose2D & step : best.poses) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = local.header;
    stamped.pose.position.x = step.x;
    stamped.pose.position.y = step.y;
    stamped.pose.orientation = quaternion_of(step.theta);
    local.poses.push_back(stamped);
  }
  local_plan_pub_->publish(local);
}

void DwaController::publish_trajectories(const DwaResult & result, const std::string & frame)
{
  visualization_msgs::msg::MarkerArray markers;
  const rclcpp::Time now = clock_->now();

  // Clear the previous cycle's candidates first.
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = frame;
  clear.header.stamp = now;
  clear.ns = "dwa";
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  int id = 0;
  for (const Trajectory & trajectory : result.trajectories) {
    if (!trajectory.valid) {
      continue;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = now;
    marker.ns = "dwa";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.color.r = 0.1;
    marker.color.g = 0.8;
    marker.color.b = 0.2;
    marker.color.a = 0.4;
    for (const Pose2D & step : trajectory.poses) {
      geometry_msgs::msg::Point point;
      point.x = step.x;
      point.y = step.y;
      point.z = 0.05;
      marker.points.push_back(point);
    }
    markers.markers.push_back(marker);
  }
  trajectories_pub_->publish(markers);
}

}  // namespace rapid_nav_controller

PLUGINLIB_EXPORT_CLASS(rapid_nav_controller::DwaController, nav2_core::Controller)
