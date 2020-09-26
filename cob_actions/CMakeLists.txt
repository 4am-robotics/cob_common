cmake_minimum_required(VERSION 3.0.2)
project(cob_actions)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs message_generation)

add_action_files(FILES
  SetBool.action
  SetFloat.action
  SetInt.action
  SetString.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib_msgs message_runtime
)
