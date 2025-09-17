cmake_minimum_required(VERSION 3.5)
project(route_executor2)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(harpia_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#[[
find_package(nav2_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
]]
set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    rclpy
    #[[
    nav2_msgs
    plansys2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    ]]
    geometry_msgs
    mavros_msgs
    lifecycle_msgs
    std_srvs
    harpia_msgs
    sensor_msgs
)

#[[
add_executable(planning_controller_node src/planning_controller_node.cpp)
ament_target_dependencies(planning_controller_node ${dependencies})
target_link_libraries(planning_controller_node nlohmann_json::nlohmann_json)


add_executable(plansys_interface src/plansys_interface.cpp)
ament_target_dependencies(plansys_interface ${dependencies})
target_link_libraries(plansys_interface nlohmann_json::nlohmann_json)
]]

install(DIRECTORY launch pddl libs data solver scripts DESTINATION share/${PROJECT_NAME})
install(DIRECTORY libs DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  #[[
  planning_controller_node
  plansys_interface
  ]]
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/route_executor.py
  scripts/path_planner.py
  scripts/data_server.py
  scripts/lifecycle_manager.py
  scripts/problem_generator.py
  scripts/mission_controller.py
  scripts/action_planner.py
  scripts/go_to.py
  scripts/take_image.py
  scripts/recharge_battery.py
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()