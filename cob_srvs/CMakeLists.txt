cmake_minimum_required(VERSION 3.0.2)
project(cob_srvs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_service_files(FILES
  SetFloat.srv
  SetInt.srv
  SetString.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)
