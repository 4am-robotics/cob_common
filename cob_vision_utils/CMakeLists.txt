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
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX /usr/local)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

include(InstallRequiredSystemLibraries)

set(CPACK_GENERATOR DEB)
SET(CPACK_SET_DESTDIR "ON")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "cob_vision_utils")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER goa@ipa.fhg.de)
#set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/Readme.txt")
#set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/Copyright.txt")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "1")
#set(CPACK_PACKAGE_VERSION_PATCH "0")

set(CPACK_PACKAGE_FILE_NAME cob_vision_utils-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}-ubuntu-10.10)
SET(CPACK_PACKAGE_NAME cob_vision_utils-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR})

#set(CPACK_DEBIAN_PACKAGE_DEPENDS
#        "libstdc++6 (>= 4.4.3-4ubuntu5),
#         libgcc1 (>= 1:4.4.3-4ubuntu5)"
#)
# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#message(STATUS ${CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES})
# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")
#message(STATUS ${CMAKE_INSTALL_RPATH})
#message(STATUS ${CMAKE_INSTALL_PREFIX})


# add project libraries
rosbuild_add_library(cob_vision_utils	common/src/VisionUtils.cpp
					common/src/CameraSensorToolbox.cpp)	
# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

# add compile flag
rosbuild_add_compile_flags(cob_vision_utils -D__LINUX__ -D__COB_ROS__)

# add project executable

# add include search paths
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

# add compile flag

# link libraries
INSTALL(TARGETS cob_vision_utils
    LIBRARY DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
)

include(CPack)
