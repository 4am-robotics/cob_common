cmake_minimum_required(VERSION 3.0.2)
project(cob_msgs)
find_package(catkin REQUIRED COMPONENTS diagnostic_msgs message_generation std_msgs)

add_message_files(FILES
  AccessPoint.msg
  DashboardState.msg
  EmergencyStopState.msg
  Network.msg
  PowerState.msg
  SiteSurvey.msg
)

generate_messages(DEPENDENCIES diagnostic_msgs std_msgs)

catkin_package(CATKIN_DEPENDS diagnostic_msgs message_runtime std_msgs)
