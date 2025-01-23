#!/usr/bin/env bash

set -e # exit on first error

cd "$(dirname "$0")/.."
if [[ `which ros2` = "" ]]
then
    echo "No ros2 binaries in PATH. Please source the setup script of your installed ROS2 distribution"
    exit -1
fi

colcon build --symlink-install
source install/setup.bash