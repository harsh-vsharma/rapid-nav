cmake_minimum_required(VERSION 3.8)
project(rapid_nav_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY maps config launch rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/ttc_governor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ttc_governor
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_maps test/test_maps.py
    TIMEOUT 240
  )
  ament_add_pytest_test(test_ttc_governor test/test_ttc_governor.py
    TIMEOUT 60
  )
endif()

ament_package()
