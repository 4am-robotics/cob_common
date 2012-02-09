cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_find_ros_package(actionlib_msgs)
#include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
#genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# rostest for robots
#rosbuild_add_roslaunch_check(robots/cob3-1.launch ROBOT=cob3-1)
#rosbuild_add_roslaunch_check(robots/cob3-2.launch ROBOT=cob3-2)
#rosbuild_add_roslaunch_check(robots/cob3-3.launch ROBOT=cob3-3)

# rostest for components
rosbuild_add_roslaunch_check(components/base.launch ROBOT=cob3-3)
rosbuild_add_roslaunch_check(components/cameras/kinect.launch ROBOT=cob3-3)
rosbuild_add_roslaunch_check(components/cameras/left_prosilica.launch ROBOT=cob3-3)
rosbuild_add_roslaunch_check(components/cameras/right_prosilica.launch ROBOT=cob3-3)
rosbuild_add_roslaunch_check(components/head.launch ROBOT=cob3-3)
rosbuild_add_roslaunch_check(components/joy.launch ROBOT=cob3-3)
rosbuild_add_roslaunch_check(components/laser_front.launch ROBOT=cob3-3)
rosbuild_add_roslaunch_check(components/laser_rear.launch ROBOT=cob3-3)
rosbuild_add_roslaunch_check(components/laser_top.launch ROBOT=cob3-3)
#rosbuild_add_roslaunch_check(components/lbr.launch ROBOT=cob3-3)
rosbuild_add_roslaunch_check(components/lwa.launch ROBOT=cob3-3)
rosbuild_add_roslaunch_check(components/relayboard.launch ROBOT=cob3-3)
rosbuild_add_roslaunch_check(components/sdh.launch ROBOT=cob3-3)
rosbuild_add_roslaunch_check(components/torso.launch ROBOT=cob3-3)
rosbuild_add_roslaunch_check(components/tray.launch ROBOT=cob3-3)


# rostest for tools
rosbuild_add_roslaunch_check(tools/dashboard.launch ROBOT=cob3-3 ROBOT_ENV=ipa-kitchen)
rosbuild_add_roslaunch_check(tools/teleop.launch ROBOT=cob3-3)
#rosbuild_add_roslaunch_check(tools/diagnostics_aggregator.launch ROBOT=cob3-3)

#rosbuild_add_roslaunch_check(ros/launch/sim.launch ROBOT=cob3-3 ROBOT_ENV=ipa-kitchen)

# Check whether there's an X display.  Intended to use in gating tests that
# require a display.
#rosbuild_check_for_display(disp)

#if(disp)
#  rosbuild_add_rostest(ros/launch/sim.launch TIMEOUT 100.0)
#endif(disp)


