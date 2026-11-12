cmake_minimum_required(VERSION 3.8)
project(rapid_nav_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY maps config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_maps test/test_maps.py
    TIMEOUT 240
  )
endif()

ament_package()
