cmake_minimum_required(VERSION 3.8)
project(rapid_nav_planner)

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
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# The search library has no ROS dependency; the planner plugin is built on top of it.
add_library(rapid_nav_grid_search SHARED src/grid_search.cpp)
target_include_directories(rapid_nav_grid_search PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(rapid_nav_planner SHARED src/grid_planner.cpp)
target_include_directories(rapid_nav_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rapid_nav_planner PUBLIC
  rapid_nav_grid_search
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  pluginlib::pluginlib
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros)
# Nav2's exported target for nav2_core; Jazzy's nav2_core is header-only and exports only its include
# directories.
if(TARGET nav2_core::nav2_core)
  target_link_libraries(rapid_nav_planner PUBLIC nav2_core::nav2_core)
else()
  target_include_directories(rapid_nav_planner PUBLIC ${nav2_core_INCLUDE_DIRS})
endif()

# The benchmark (no ROS): the map loader, the cost grid, the random queries and the timing.
add_library(rapid_nav_bench SHARED src/bench.cpp)
target_include_directories(rapid_nav_bench PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rapid_nav_bench rapid_nav_grid_search)

add_executable(planner_bench bench/planner_bench.cpp)
target_link_libraries(planner_bench rapid_nav_bench)

install(TARGETS rapid_nav_grid_search rapid_nav_planner rapid_nav_bench
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS planner_bench RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
pluginlib_export_plugin_description_file(nav2_core grid_planner_plugin.xml)
ament_export_include_directories(include)
ament_export_libraries(rapid_nav_grid_search rapid_nav_planner rapid_nav_bench)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util pluginlib nav_msgs geometry_msgs std_msgs tf2 tf2_ros)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grid_search test/test_grid_search.cpp)
  target_link_libraries(test_grid_search rapid_nav_grid_search)
  ament_add_gtest(test_bench test/test_bench.cpp)
  target_link_libraries(test_bench rapid_nav_bench)

  find_package(ament_lint_auto REQUIRED)
  # Only the C++ style check (line length 100), the XML and CMake lints; no copyright, cppcheck or uncrustify.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cppcheck ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
