cmake_minimum_required(VERSION 3.8)
project(rt2_navigation_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# Generate C++/Python type support for the custom navigation action.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToPose.action"
)

# Runtime export is required by packages that include the generated action headers.
ament_export_dependencies(rosidl_default_runtime)
ament_package()
