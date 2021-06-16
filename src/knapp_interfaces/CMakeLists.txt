cmake_minimum_required(VERSION 3.5)
project(knapp_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Werror -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################
set(MSG_LIST
  "msg/NextOrder.msg"
)

set (DEPENDENCIES_LIST
  "std_msgs"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_LIST}
  DEPENDENCIES ${DEPENDENCIES_LIST}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rosidl_default_runtime)
ament_package()