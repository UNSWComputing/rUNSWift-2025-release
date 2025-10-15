#!/bin/bash

# dependencies installation script

REALPATH="$(realpath "$0")"
SCRIPTS_DIR="$(dirname "$REALPATH")"
SRC_DIR=$SCRIPTS_DIR/../../../src

$SCRIPTS_DIR/pre-vcs.sh
vcs import $SRC_DIR < $SRC_DIR/motion/ros_motion/dependencies.repos --recursive
# remove nao_lola package as it clashes with image
rm -rf $SRC_DIR/3rdparty/src/nao_lola/nao_lola

$SCRIPTS_DIR/post-vcs.sh

$SCRIPTS_DIR/pre-rosdep.sh
rosdep install --from-paths $SRC_DIR --ignore-src -r -y
$SCRIPTS_DIR/post-rosdep.sh
