cmake_minimum_required(VERSION 3.8)
project(cob_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Assuming your message files are now located in the "msg" directory.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AccessPoint.msg"
  "msg/DashboardState.msg"
  "msg/EmergencyStopState.msg"
  "msg/Network.msg"
  "msg/PowerState.msg"
  "msg/SafetyControllerState.msg"
  "msg/SiteSurvey.msg"
  DEPENDENCIES diagnostic_msgs std_msgs
)

ament_package()
