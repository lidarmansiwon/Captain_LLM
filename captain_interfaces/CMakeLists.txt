cmake_minimum_required(VERSION 3.8)
project(captain_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(MSG_FILES
  "msg/CaptainOrder.msg"
  "msg/CaptainResponse.msg"
  "msg/ShipStatus.msg"


  "msg/Pose2D.msg"
  "msg/Velocity2D.msg"
  "msg/Health.msg"
  "msg/Point2D.msg"
  "msg/Polygon.msg"
  "msg/EgoState.msg"
  "msg/Target.msg"
  "msg/Hazard.msg"
  "msg/Env.msg"
  "msg/WorldState.msg"
  "msg/Limits.msg"
  "msg/Constraints.msg"
  "msg/PlannerRequest.msg"
  "msg/Action.msg"
  "msg/HighLevelPlan.msg"
  "msg/PlanDecision.msg"
  "msg/Trajectory.msg"
  "msg/Event.msg"
)

set(SRV_FILES
  "srv/ExecuteOrder.srv"
  "srv/MakePlan.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()