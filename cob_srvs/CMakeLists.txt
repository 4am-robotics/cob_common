cmake_minimum_required(VERSION 3.0.2)
project(cob_srvs)

find_package(catkin REQUIRED COMPONENTS cob_msgs geometry_msgs message_generation)

add_service_files(FILES
  Dock.srv
  SetFloat.srv
  SetHornMode.srv
  SetInt.srv
  SetString.srv
)

generate_messages(
  DEPENDENCIES cob_msgs geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS cob_msgs geometry_msgs message_runtime
)
