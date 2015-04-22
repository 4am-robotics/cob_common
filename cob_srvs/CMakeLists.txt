cmake_minimum_required(VERSION 2.8.3)
project(cob_srvs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_service_files(FILES
  Trigger.srv
  SetFloat.srv
  SetInt.srv
  SetString.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs message_runtime
)
