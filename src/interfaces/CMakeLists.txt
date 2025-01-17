cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Battery.msg"
  "msg/CartesianPoint.msg"
  "msg/ChangeMissionAceppet.msg"
  "msg/DronePose.msg"
  "msg/Frame.msg"
  "msg/Goal.msg"
  "msg/SafetyParams.msg"
  "msg/Tuple2D.msg"
  "msg/Camera.msg"
  "msg/GeoPoint.msg"
  "msg/UAV.msg"
  "msg/Point.msg"
  "msg/Region.msg"
  "msg/Map.msg"
  "msg/RegionPoint.msg"
  "msg/ChangeMission.msg"
  "msg/Mission.msg"
  "msg/ActionDispatch.msg"
  "msg/CompletePlan.msg"
  "msg/GoalID.msg"
  "srv/PathPlanning.srv"
  "srv/MissionFaultMitigation.srv"
  "srv/MissionGoalManager.srv"
  "srv/WeatherCheck.srv"
  "action/MissionPlanner.action"
  DEPENDENCIES geometry_msgs mavros_msgs diagnostic_msgs builtin_interfaces # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_package()

