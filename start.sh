#!/bin/bash

echo "Doing docker trick..."
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/docker_trick.xml
. install/setup.bash
ros2 run pose_estimation solver /asset/config.yaml