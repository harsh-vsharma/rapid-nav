cmake_minimum_required(VERSION 3.8)
project(rapid_nav_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FlowFrame.msg"
  DEPENDENCIES std_msgs sensor_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # XML lint only; the message file has no code to lint.
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
