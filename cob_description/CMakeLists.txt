cmake_minimum_required(VERSION 2.4.6)
project(cob_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

#catkin_add_gtest(test_urdf test/test_urdf.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})



install(DIRECTORY urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY gazebo
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY Media
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS test/test_urdf.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/test
)
