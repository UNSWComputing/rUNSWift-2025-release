#!/bin/bash

# Actions to be performed before rosdep install
REALPATH="$(realpath "$0")"
SCRIPTS_DIR="$(dirname "$REALPATH")"
SRC_DIR=$SCRIPTS_DIR/../src

echo "**pre-rosdep: start**"

# Check ROS_DISTRO
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS_DISTRO is not set. Make sure you source /opt/ros/humble/setup.bash before running again."
else
    echo "ROS_DISTRO is set to '$ROS_DISTRO'."
fi

echo "**pre-rosdep: nao**"
if [ ! -d "$SRC_DIR/nao/nao_description/meshes" ]
then 
    $SRC_DIR/nao/nao_description/install.sh
fi

echo "**pre-rosdep: end**"
