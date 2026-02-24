#!/bin/bash

export ROS_IP=`hostname -I | cut -d ' ' -f1`

source /root/catkin_ws/devel/setup.bash
/opt/ros/melodic/bin/roslaunch simple_talker simple_talker.launch
