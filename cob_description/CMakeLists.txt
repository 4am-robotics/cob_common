cmake_minimum_required(VERSION 2.8.3)
project(cob_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY gazebo Media meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS test/test_urdf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/test
)
