cmake_minimum_required(VERSION 3.8)
project(rapid_nav_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# The search library has no ROS dependency; the planner plugin (grid_planner) is built on top of it.
add_library(rapid_nav_grid_search SHARED src/grid_search.cpp)
target_include_directories(rapid_nav_grid_search PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS rapid_nav_grid_search
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_libraries(rapid_nav_grid_search)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grid_search test/test_grid_search.cpp)
  target_link_libraries(test_grid_search rapid_nav_grid_search)

  find_package(ament_lint_auto REQUIRED)
  # Only the C++ style check (line length 100), the XML and CMake lints; no copyright, cppcheck or uncrustify.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cppcheck ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
