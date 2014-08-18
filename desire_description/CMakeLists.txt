cmake_minimum_required(VERSION 2.4.6)
project(desire_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#catkin_add_gtest(test_urdf_desire test/test_urdf.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
