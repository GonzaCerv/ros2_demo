cmake_minimum_required(VERSION 3.5)
project(knapp_amr)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(knapp_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)

# includes
include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
)

###########
## Build ##
###########
add_library(travel_salesman
  src/pathFinder/travelSalesman/algorithms.cpp
  src/pathFinder/travelSalesman/city.cpp
  src/pathFinder/travelSalesman/tree.cpp
)

add_executable(order_optimizer 
  src/OrderOptimizer.cpp
  src/OrderOptimizerRunner.cpp
  src/OrderFinder.cpp)
target_link_libraries(order_optimizer 
  yaml-cpp
  travel_salesman
)
ament_target_dependencies(order_optimizer
  rclcpp
  geometry_msgs
  knapp_interfaces
  std_msgs
  visualization_msgs
)

#############
## Install ##
#############
install(
  TARGETS order_optimizer
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY launch yaml
  DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_order_finder
    src/OrderFinder.cpp
    test/unit_test/OrderFinderTest.cpp
  )
  target_link_libraries(${PROJECT_NAME}_order_finder
  yaml-cpp
  )

  ament_add_gtest(${PROJECT_NAME}_configuration_file
  test/unit_test/ConfigurationFileTest.cpp
  )
  target_link_libraries(${PROJECT_NAME}_configuration_file
  yaml-cpp
  )
endif()

ament_package()
