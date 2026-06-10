cmake_minimum_required(VERSION 3.8)
project(araseo_navigation)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToNearestStop.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_pytest_test(test_route_data tests/test_route_data.py)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY action launch params rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  nodes/nearest_stop_navigator.py
  scripts/rviz_display_route_marker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
