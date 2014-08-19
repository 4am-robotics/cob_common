cmake_minimum_required(VERSION 2.8.3)
project(cob_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
  FILES
  AccessPoint.msg
  BatteryServer2.msg
  BatteryServer.msg
  BatteryState2.msg
  BatteryState.msg
  DashboardState.msg
  PowerBoardState.msg
  PowerState.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs)

