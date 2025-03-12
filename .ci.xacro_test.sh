#!/bin/sh
status=0
# Only set --inorder if running on older ROS 1 distros; for ROS 2 (Humble) we don't need it.
if [ -n "$ROS_DISTRO" ] && { [ "$ROS_DISTRO" = "indigo" ] || [ "$ROS_DISTRO" = "jade" ] || [ "$ROS_DISTRO" = "kinetic" ]; }; then
    xacro_args='--inorder'
else
    xacro_args=''
fi

for x in $(find "$TARGET_REPO_PATH" -name '*.xacro' | sort); do
    echo "Testing $x"
    xacro $xacro_args "$x" > /dev/null || status=1
done
exit $status
