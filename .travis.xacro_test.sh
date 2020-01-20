#!/bin/sh
status=0
[ "$ROS_DISTRO" \< "melodic" ] && xacro_args='--inorder'
for x in $(find "$TARGET_REPO_PATH" -name '*.xacro'|sort); do
    echo "Testing $x"
    xacro $xacro_args "$x" > /dev/null || status=1
done
exit $status
