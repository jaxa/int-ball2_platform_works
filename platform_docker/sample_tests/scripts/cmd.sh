#!/bin/bash

export ROS_IP=`hostname -I | cut -d ' ' -f1`

source /root/catkin_ws/devel/setup.bash

# select the launch file
/opt/ros/noetic/bin/roslaunch sample_tests simple_test.launch
# /opt/ros/noetic/bin/roslaunch sample_tests test_fan_duty.launch