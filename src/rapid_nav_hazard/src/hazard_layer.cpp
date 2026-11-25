// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).

#include "rapid_nav_hazard/hazard_layer.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rapid_nav_hazard/compat.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.hpp"

namespace rapid_nav_hazard
{

void HazardLayer::onInitialize()
{
  const auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("HazardLayer: the parent node has expired");
  }
  // Nav2 1.5 dropped Layer::declareParameter; declare under the same full name ("<layer>.<param>").
  const auto declare = [&](const std::string & param, const rclcpp::ParameterValue & value) {
#if RAPID_NAV_NAV2_ROS_COMMON
      nav2::declare_parameter_if_not_declared(node, name_ + "." + param, value);
#else
      declareParameter(param, value);
#endif
    };
  declare("enabled", rclcpp::ParameterValue(true));
  declare("topic", rclcpp::ParameterValue(std::string("/obstacles/tracked")));
  declare("horizon_s", rclcpp::ParameterValue(params_.horizon_s));
  declare("time_step_s", rclcpp::ParameterValue(params_.time_step_s));
  declare("robot_radius", rclcpp::ParameterValue(params_.robot_radius));
  declare("margin", rclcpp::ParameterValue(params_.margin));
  declare("max_cost", rclcpp::ParameterValue(params_.max_cost));
  declare("min_cost", rclcpp::ParameterValue(params_.min_cost));
  declare("min_speed", rclcpp::ParameterValue(params_.min_speed));
  declare("max_age_s", rclcpp::ParameterValue(params_.max_age_s));

  std::string topic;
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic);
  node->get_parameter(name_ + ".horizon_s", params_.horizon_s);
  node->get_parameter(name_ + ".time_step_s", params_.time_step_s);
  node->get_parameter(name_ + ".robot_radius", params_.robot_radius);
  node->get_parameter(name_ + ".margin", params_.margin);
  node->get_parameter(name_ + ".max_cost", params_.max_cost);
  node->get_parameter(name_ + ".min_cost", params_.min_cost);
  node->get_parameter(name_ + ".min_speed", params_.min_speed);
  node->get_parameter(name_ + ".max_age_s", params_.max_age_s);
  // A prediction must never be as bad as a wall (253 and up): cap whatever the parameter says.
  params_.max_cost = std::min(
    params_.max_cost, static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) - 1);
  params_.min_cost = std::min(params_.min_cost, params_.max_cost);

  sub_ = compat::create_subscription<rapid_nav_msgs::msg::TrackedObstacleArray>(
    node, topic, rclcpp::QoS(1).reliable(),
    [this](rapid_nav_msgs::msg::TrackedObstacleArray::ConstSharedPtr msg) {on_message(msg);});
  current_ = true;
  RCLCPP_INFO(logger_, "HazardLayer '%s' listens on %s", name_.c_str(), topic.c_str());
}

void HazardLayer::on_message(rapid_nav_msgs::msg::TrackedObstacleArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = std::move(msg);
}

void HazardLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.reset();
  disks_.clear();
  last_box_ = BoundingBox();
}

void HazardLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  disks_.clear();
  if (!enabled_) {
    return;
  }

  rapid_nav_msgs::msg::TrackedObstacleArray::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg = latest_;
  }

  if (msg) {
    const double age = (clock_->now() - rclcpp::Time(msg->header.stamp, clock_->get_clock_type()))
      .seconds();
    if (age <= params_.max_age_s) {
      const std::string global_frame = layered_costmap_->getGlobalFrameID();
      tf2::Transform transform;
      transform.setIdentity();
      bool ok = true;
      if (msg->header.frame_id != global_frame) {
        try {
          const auto t = tf_->lookupTransform(
            global_frame, msg->header.frame_id, tf2::TimePointZero);
          const auto & q = t.transform.rotation;
          const auto & d = t.transform.translation;
          transform = tf2::Transform(
            tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(d.x, d.y, d.z));
        } catch (const std::exception & e) {
          ok = false;
          RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000, "HazardLayer: no transform %s -> %s: %s",
            msg->header.frame_id.c_str(), global_frame.c_str(), e.what());
        }
      }
      if (ok) {
        std::vector<Obstacle> obstacles;
        for (const auto & o : msg->obstacles) {
          const tf2::Vector3 p = transform * tf2::Vector3(o.position.x, o.position.y, 0.0);
          // Velocity: rotation only, no translation.
          const tf2::Vector3 v = tf2::Transform(transform.getRotation()) *
            tf2::Vector3(o.velocity.x, o.velocity.y, 0.0);
          Obstacle ob;
          ob.x = p.x();
          ob.y = p.y();
          ob.vx = v.x();
          ob.vy = v.y();
          ob.radius = o.radius;
          // Per-axis sigmas do not change under a rotation.
          ob.position_sigma = o.position_sigma;
          ob.velocity_sigma = o.velocity_sigma;
          obstacles.push_back(ob);
        }
        disks_ = predict_disks(obstacles, std::max(age, 0.0), params_);
      }
    }
  }

  const BoundingBox now_box = bounding_box(disks_);
  for (const BoundingBox & box : {now_box, last_box_}) {
    if (!box.empty) {
      *min_x = std::min(*min_x, box.min_x);
      *min_y = std::min(*min_y, box.min_y);
      *max_x = std::max(*max_x, box.max_x);
      *max_y = std::max(*max_y, box.max_y);
    }
  }
  last_box_ = now_box;
}

void HazardLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || disks_.empty()) {
    return;
  }
  GridSpec grid;
  grid.origin_x = master_grid.getOriginX();
  grid.origin_y = master_grid.getOriginY();
  grid.resolution = master_grid.getResolution();
  grid.size_x = static_cast<int>(master_grid.getSizeInCellsX());
  grid.size_y = static_cast<int>(master_grid.getSizeInCellsY());
  for (const auto & c : rasterise(disks_, grid)) {
    if (c.i < min_i || c.i >= max_i || c.j < min_j || c.j >= max_j) {
      continue;
    }
    const unsigned char old = master_grid.getCost(c.i, c.j);
    // Raise only: an unknown cell (255) counts as free here.
    if (old == nav2_costmap_2d::NO_INFORMATION || old < c.cost) {
      master_grid.setCost(c.i, c.j, c.cost);
    }
  }
}

}  // namespace rapid_nav_hazard

PLUGINLIB_EXPORT_CLASS(rapid_nav_hazard::HazardLayer, nav2_costmap_2d::Layer)
