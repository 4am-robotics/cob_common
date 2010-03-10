cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/common/lib)

#uncomment if you have defined messages
#genmsg()
#uncomment if you have defined services
#gensrv()

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# add project libraries
rosbuild_add_library(cob_vision_utils	common/src/MathUtils.cpp
					common/src/OpenCVUtils.cpp)	
# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

# add compile flag
rosbuild_add_compile_flags(cob_vision_utils -D__LINUX__ -D__COB_ROS__)

# add project executable

# add include search paths
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

# add compile flag

# link libraries
target_link_libraries(cob_vision_utils cv cxcore highgui Wm4Foundation)
