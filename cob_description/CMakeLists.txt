cmake_minimum_required(VERSION 3.5)
project(cob_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY Media meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS test/test_urdf.py
  DESTINATION share/${PROJECT_NAME}/test
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_package()
