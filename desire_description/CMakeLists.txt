cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(catkin REQUIRED COMPONENTS)
#catkin_package(CATKIN_DEPENDS xacro)
catkin_package(CATKIN_DEPENDS)

#catkin_add_gtest(test_urdf_desire test/test_urdf.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})


