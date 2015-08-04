cmake_minimum_required(VERSION 2.4.6)
project(cob_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  CATKIN_DEPENDS gazebo_ros rospy rosunit
)

install(DIRECTORY gazebo Media meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS test/test_urdf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/test
)
