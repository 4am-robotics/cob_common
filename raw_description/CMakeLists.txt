cmake_minimum_required(VERSION 2.4.6)
project(raw_description)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS cob_description gazebo_ros
)

install(DIRECTORY Media meshes urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
