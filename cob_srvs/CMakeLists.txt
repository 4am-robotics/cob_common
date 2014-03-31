cmake_minimum_required(VERSION 2.8.3)
project(cob_srvs)

find_package(catkin REQUIRED COMPONENTS std_msgs trajectory_msgs geometry_msgs message_generation)


#uncomment if you have defined services
add_service_files(FILES
  Trigger.srv
  SetMaxVel.srv
  SetInt.srv
  SetFloat.srv
  SetString.srv
  SetOperationMode.srv
  SetJointTrajectory.srv
  SetJointStiffness.srv
  SetDefaultVel.srv
  GetPoseStampedTransformed.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs trajectory_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs trajectory_msgs geometry_msgs
)
