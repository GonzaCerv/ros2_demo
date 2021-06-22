cmake_minimum_required(VERSION 3.5)
project(goal_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_demo_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_component SHARED
  src/goal_controller_action_server.cpp)
rclcpp_components_register_node(${PROJECT_NAME}_component PLUGIN "goal_controller::GoalControllerServer" EXECUTABLE goal_controller_action_server)
ament_target_dependencies(${PROJECT_NAME}_component
  "geometry_msgs"
  "turtlesim"
  "ros_demo_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")


# Install packages.
install(TARGETS
  ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
