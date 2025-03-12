cmake_minimum_required(VERSION 3.8)
project(cob_srvs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Assuming your service files are now in the "srv" directory.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Dock.srv"
  "srv/SetFloat.srv"
  "srv/SetInt.srv"
  "srv/SetString.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
