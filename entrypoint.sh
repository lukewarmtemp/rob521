#!/bin/bash

set -e

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

cd /catkin_ws

echo "Provided arguments: $@"

exec $@