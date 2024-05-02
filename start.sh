#!/bin/bash

echo "Doing docker trick..."
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/docker_trick.xml
. /opt/ros/humble/setup.bash
. install/setup.bash
ros2 run pose_estimation solver /asset/config.yaml