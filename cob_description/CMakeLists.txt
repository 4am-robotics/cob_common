cmake_minimum_required(VERSION 3.0.2)
project(cob_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY Media meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS test/test_urdf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
