cmake_minimum_required(VERSION 3.5)
project(mission_planning)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Uncomment if the package has a setup.py. This macro ensures modules and global scripts get installed
# ament_python_install_package(${PROJECT_NAME})

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${action_msgs_INCLUDE_DIRS}
  ${geographic_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${interfaces_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# Declare a C++ executable
# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

# Specify libraries to link a library or executable target against
# ament_target_dependencies(${PROJECT_NAME}_node
#   rclcpp
#   action_msgs
#   geographic_msgs
#   geometry_msgs
#   interfaces
#   std_msgs
# )

# Uncomment if you have a library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mission_planning.cpp
# )

# Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}
#   ${rclcpp_LIBRARIES}
# )

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
