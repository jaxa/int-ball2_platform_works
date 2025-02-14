#!/bin/bash

DIR_ROS_BASE=/opt/ros/melodic
DIR_CATKIN_WS=/home/nvidia/IB2
ENV_FILE_ROS_HOME=/etc/flight_software/ros_home

source ${DIR_ROS_BASE}/setup.bash
source ${DIR_CATKIN_WS}/devel/setup.bash
source ${ENV_FILE_ROS_HOME}

rosrun platform_manager ib2_bringup.sh &> /dev/null

