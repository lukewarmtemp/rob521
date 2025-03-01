#!/bin/bash

set -e

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash || true

cd /catkin_ws

echo "Provided arguments: $@"

exec $@