#!/bin/bash
status=0
for x in $(find "$TARGET_REPO_PATH" -name '*.xacro'|sort); do
    echo "Testing $x"
    xacro --inorder "$x" > /dev/null || status=1
done
exit $status
