// Copyright (c) 2026 RAPID-NAV team. MIT licence (see LICENSE).
//
// Nav2 API differences between ROS 2 Jazzy (Nav2 1.3) and Lyrical (Nav2 1.5).
// From Nav2 1.4 on, plugins receive nav2::LifecycleNode (package nav2_ros_common) instead of
// rclcpp_lifecycle::LifecycleNode, and the parameter helpers moved to the nav2 namespace.
#ifndef RAPID_NAV_PLANNER__COMPAT_HPP_
#define RAPID_NAV_PLANNER__COMPAT_HPP_

#if __has_include("nav2_ros_common/lifecycle_node.hpp")
#define RAPID_NAV_NAV2_ROS_COMMON 1
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"
#else
#define RAPID_NAV_NAV2_ROS_COMMON 0
#include "nav2_util/node_utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#endif

namespace rapid_nav_planner::compat
{

#if RAPID_NAV_NAV2_ROS_COMMON
using LifecycleNode = nav2::LifecycleNode;
using nav2::declare_parameter_if_not_declared;
#else
using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
using nav2_util::declare_parameter_if_not_declared;
#endif

}  // namespace rapid_nav_planner::compat

#endif  // RAPID_NAV_PLANNER__COMPAT_HPP_
