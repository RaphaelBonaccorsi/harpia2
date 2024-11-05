cmake_minimum_required(VERSION 3.5)
project(route_executor2)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
#[[
find_package(nav2_msgs REQUIRED)
]]
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)

find_package(nlohmann_json REQUIRED) # sudo apt-get install nlohmann-json3-dev

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    rclpy
    plansys2_msgs
    #[[
    nav2_msgs
    ]]
    geometry_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    mavros_msgs
    lifecycle_msgs
)

add_executable(planning_controller_node src/planning_controller_node.cpp)
ament_target_dependencies(planning_controller_node ${dependencies})
target_link_libraries(planning_controller_node nlohmann_json::nlohmann_json)


add_executable(go_to_action_node src/go_to_action_node.cpp)
ament_target_dependencies(go_to_action_node rclcpp plansys2_executor)
add_executable(take_image_action_node src/take_image_action_node.cpp)
ament_target_dependencies(take_image_action_node rclcpp plansys2_executor)
add_executable(pulverize_region_action_node src/pulverize_region_action_node.cpp)
ament_target_dependencies(pulverize_region_action_node rclcpp plansys2_executor)
add_executable(recharge_battery_action_node src/recharge_battery_action_node.cpp)
ament_target_dependencies(recharge_battery_action_node rclcpp plansys2_executor)
add_executable(recharge_input_action_node src/recharge_input_action_node.cpp)
ament_target_dependencies(recharge_input_action_node rclcpp plansys2_executor)

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  planning_controller_node
  go_to_action_node
  take_image_action_node
  pulverize_region_action_node
  recharge_battery_action_node
  recharge_input_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
