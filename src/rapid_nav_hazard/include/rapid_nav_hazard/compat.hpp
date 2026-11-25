// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).
//
// Nav2 API differences between ROS 2 Jazzy (Nav2 1.3) and Lyrical (Nav2 1.5).
// From Nav2 1.4 on, costmap layers receive nav2::LifecycleNode (package nav2_ros_common) instead
// of rclcpp_lifecycle::LifecycleNode; its create_subscription takes the callback before the
// QoS, and Layer::declareParameter is gone (nav2::declare_parameter_if_not_declared replaces it).
#ifndef RAPID_NAV_HAZARD__COMPAT_HPP_
#define RAPID_NAV_HAZARD__COMPAT_HPP_

#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#if __has_include("nav2_ros_common/lifecycle_node.hpp")
#define RAPID_NAV_NAV2_ROS_COMMON 1
#include "nav2_ros_common/node_utils.hpp"
#else
#define RAPID_NAV_NAV2_ROS_COMMON 0
#endif

namespace rapid_nav_hazard::compat
{

template<typename MessageT, typename NodeT, typename CallbackT>
typename rclcpp::Subscription<MessageT>::SharedPtr create_subscription(
  const NodeT & node, const std::string & topic, const rclcpp::QoS & qos, CallbackT && callback)
{
#if RAPID_NAV_NAV2_ROS_COMMON
  return node->template create_subscription<MessageT>(
    topic, std::forward<CallbackT>(callback), qos);
#else
  return node->template create_subscription<MessageT>(
    topic, qos, std::forward<CallbackT>(callback));
#endif
}

}  // namespace rapid_nav_hazard::compat

#endif  // RAPID_NAV_HAZARD__COMPAT_HPP_
