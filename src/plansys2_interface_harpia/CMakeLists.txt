cmake_minimum_required(VERSION 3.5)

project(plansys2_interface_harpia)

## Compile as C++17, recomendado para ROS 2
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
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(interfaces REQUIRED)

## Declare ament package
include_directories(
  include
  ${interfaces_INCLUDE_DIRS}  # Inclua os diretórios de cabeçalhos para o pacote interfaces
)

## Declare action interface for HarpiaExecutor
add_executable(HarpiaExecutor src/RPHarpiaExecutor.cpp)
ament_target_dependencies(HarpiaExecutor
  rclcpp
  action_msgs
  std_msgs
  std_srvs
  diagnostic_msgs
  plansys2_executor
  plansys2_msgs
  interfaces
)

install(TARGETS
  HarpiaExecutor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

ament_export_include_directories(include ${interfaces_INCLUDE_DIRS})
ament_export_dependencies(
  rclcpp
  action_msgs
  std_msgs
  std_srvs
  diagnostic_msgs
  plansys2_executor
  plansys2_msgs
  interfaces
)

ament_package()