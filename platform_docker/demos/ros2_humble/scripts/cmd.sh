#!/bin/bash

export ROS_MASTER_URI=http://172.17.0.1:11311
export ROS_IP=`hostname -I | cut -d ' ' -f1`

/root/scripts/launch_simple_listener.sh &
/root/scripts/launch_ros1_bridge.sh &
/root/scripts/launch_simple_talker_ros2.sh &

wait 
