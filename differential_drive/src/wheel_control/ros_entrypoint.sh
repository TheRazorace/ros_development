#!/bin/bash

set -e

. /opt/ros/noetic/setup.bash
. /catkin_ws/devel/setup.bash

# Rosrun command
#rosrun wheel_control cmd_vel_subscriber

# Keep the container running 
exec "$@"