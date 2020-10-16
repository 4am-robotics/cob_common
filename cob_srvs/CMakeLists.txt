cmake_minimum_required(VERSION 3.0.2)
project(cob_srvs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation)

add_service_files(FILES
  Dock.srv
  SetFloat.srv
  SetInt.srv
  SetString.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs message_runtime
)
