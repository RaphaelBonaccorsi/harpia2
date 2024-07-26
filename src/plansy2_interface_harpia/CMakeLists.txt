cmake_minimum_required(VERSION 3.5)

project(rosplan_interface_harpia)

## Compile as C++14, recomendado para ROS 2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
##find_package(rosplan_planning_system REQUIRED)
find_package(interfaces REQUIRED)

###########
## Build ##
###########

include_directories(include)

## Declare action interface for RPHarpiaExecutor
add_executable(RPHarpiaExecutor src/RPHarpiaExecutor.cpp)
ament_target_dependencies(RPHarpiaExecutor
  rclcpp
  action_msgs
  std_msgs
  std_srvs
  diagnostic_msgs
  ##rosplan_planning_system
  interfaces
)

install(TARGETS
  RPHarpiaExecutor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  action_msgs
  std_msgs
  std_srvs
  diagnostic_msgs
  ##rosplan_planning_system
  interfaces
)

## Declare ament package
ament_package()