#!/bin/bash

source ${IB2_WORKSPACE}/devel/setup.bash
export ROS_IP=`hostname -I | cut -d ' ' -f1`
/opt/ros/noetic/bin/roslaunch ${IB2_PACKAGE} ${IB2_LAUNCH_FILE}
