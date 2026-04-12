cmake_minimum_required(VERSION 3.8)
project(araseo_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(araseo_bt_nodes SHARED
  src/log_custom_bt_loaded.cpp
)

add_library(araseo_lane_graph_planner SHARED
  src/road_driving_planner.cpp
)

add_library(araseo_road_driving_controller SHARED
  src/road_driving_controller.cpp
)

add_executable(lane_graph_visualizer
  src/lane_graph_visualizer.cpp
)

target_compile_features(araseo_bt_nodes PUBLIC cxx_std_17)
target_compile_features(araseo_lane_graph_planner PUBLIC cxx_std_17)
target_compile_features(araseo_road_driving_controller PUBLIC cxx_std_17)
target_compile_features(lane_graph_visualizer PUBLIC cxx_std_17)
target_compile_definitions(araseo_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(araseo_bt_nodes
  behaviortree_cpp
  rclcpp
)

ament_target_dependencies(araseo_lane_graph_planner
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_target_dependencies(araseo_road_driving_controller
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_target_dependencies(lane_graph_visualizer
  geometry_msgs
  nav_msgs
  rclcpp
  visualization_msgs
)

target_link_libraries(araseo_lane_graph_planner
  yaml-cpp
)

target_link_libraries(araseo_road_driving_controller
  yaml-cpp
)

target_link_libraries(lane_graph_visualizer
  yaml-cpp
)

target_include_directories(araseo_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(araseo_lane_graph_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(araseo_road_driving_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(lane_graph_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(nav2_core planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core controller_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  araseo_bt_nodes
  araseo_lane_graph_planner
  araseo_road_driving_controller
  lane_graph_visualizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  planner_plugin.xml
  controller_plugin.xml
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch params rviz behavior_trees include scripts
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
