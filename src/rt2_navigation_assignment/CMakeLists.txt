cmake_minimum_required(VERSION 3.8)
project(rt2_navigation_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rt2_navigation_interfaces REQUIRED)

# Action server component: reads odometry, publishes TF/cmd_vel, and executes goals.
add_library(navigation_action_server_component SHARED
  src/navigation_action_server.cpp
)

target_compile_features(navigation_action_server_component PUBLIC cxx_std_17)

ament_target_dependencies(navigation_action_server_component
  rclcpp
  rclcpp_action
  rclcpp_components
  builtin_interfaces
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rt2_navigation_interfaces
)

rclcpp_components_register_node(navigation_action_server_component
  PLUGIN "rt2_navigation_assignment::NavigationActionServer"
  EXECUTABLE navigation_action_server
)

# Client component: terminal UI plus ROS 2 action client callbacks.
add_library(navigation_client_component SHARED
  src/navigation_client_component.cpp
)

target_compile_features(navigation_client_component PUBLIC cxx_std_17)

ament_target_dependencies(navigation_client_component
  rclcpp
  rclcpp_action
  rclcpp_components
  rt2_navigation_interfaces
)

rclcpp_components_register_node(navigation_client_component
  PLUGIN "rt2_navigation_assignment::NavigationClientComponent"
  EXECUTABLE navigation_client
)

install(TARGETS
  navigation_action_server_component
  navigation_client_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  navigation_action_server
  navigation_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Export component targets and dependencies so downstream packages can load/reuse them.
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_components
  builtin_interfaces
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rt2_navigation_interfaces
)

ament_package()
