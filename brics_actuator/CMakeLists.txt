cmake_minimum_required(VERSION 2.8.3)
project(brics_actuator)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs message_generation)

add_message_files(
  DIRECTORY msg
  FILES
  CartesianPose.msg    
  JointAccelerations.msg  
  JointTorques.msg
  CartesianTwist.msg   
  JointConstraint.msg     
  JointValue.msg
  CartesianVector.msg  
  JointImpedances.msg     
  JointVelocities.msg
  CartesianWrench.msg  
  JointPositions.msg      
  Poison.msg
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
    INCLUDE_DIRS
)
