cmake_minimum_required(VERSION 3.5)
project(raw_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY Media meshes urdf
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
