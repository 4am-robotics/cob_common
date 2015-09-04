cmake_minimum_required(VERSION 2.8.3)
project(raw_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY Media meshes urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
