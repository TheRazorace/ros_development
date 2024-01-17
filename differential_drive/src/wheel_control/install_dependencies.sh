#!/bin/bash

apt-get update -y 
apt-get install -y \
    build-essential
apt-get install -y ros-noetic-catkin python3-catkin-tools 
apt-get install -y ros-noetic-geometry-msgs 
apt-get install -y ros-noetic-nav-msgs 
apt-get install -y ros-noetic-roscpp 
apt-get install -y ros-noetic-rospy 
apt-get install -y ros-noetic-sensor-msgs
apt-get install -y ros-noetic-std-msgs 
apt-get install -y ros-noetic-turtlesim