cmake_minimum_required(VERSION 3.8)
project(cob_actions)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)  # ROS 2 uses built-in action types

# Generate action interfaces.
# Make sure your action files are in the "action" directory.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Dock.action"
  "action/SetBool.action"
  "action/SetFloat.action"
  "action/SetInt.action"
  "action/SetString.action"
  DEPENDENCIES geometry_msgs action_msgs
)

ament_package()
