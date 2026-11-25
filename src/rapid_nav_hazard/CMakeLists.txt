cmake_minimum_required(VERSION 3.8)
project(rapid_nav_hazard)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # GCC on ARM notes an ABI change for std::pair arguments in C++17; it is not about this code.
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
    add_compile_options(-Wno-psabi)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rapid_nav_msgs REQUIRED)

# The prediction and rasterisation library has no ROS dependency; the costmap layer is built on top of it.
add_library(rapid_nav_hazard_core SHARED src/hazard_core.cpp)
target_include_directories(rapid_nav_hazard_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(rapid_nav_hazard_layer SHARED src/hazard_layer.cpp)
target_include_directories(rapid_nav_hazard_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rapid_nav_hazard_layer PUBLIC
  rapid_nav_hazard_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  nav2_costmap_2d::nav2_costmap_2d_core
  pluginlib::pluginlib
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${rapid_nav_msgs_TARGETS})

install(TARGETS rapid_nav_hazard_core rapid_nav_hazard_layer
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
pluginlib_export_plugin_description_file(nav2_costmap_2d hazard_layer_plugin.xml)
ament_export_include_directories(include)
ament_export_libraries(rapid_nav_hazard_core rapid_nav_hazard_layer)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_lifecycle nav2_costmap_2d pluginlib geometry_msgs tf2 tf2_ros
  tf2_geometry_msgs rapid_nav_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_hazard_core test/test_hazard_core.cpp)
  target_link_libraries(test_hazard_core rapid_nav_hazard_core)

  find_package(ament_lint_auto REQUIRED)
  # Only the C++ style check (line length 100), the XML and CMake lints; no copyright, cppcheck or uncrustify.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cppcheck ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
