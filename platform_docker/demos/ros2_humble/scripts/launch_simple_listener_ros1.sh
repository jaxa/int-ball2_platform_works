#!/bin/bash

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

rosparam load /root/catkin_ws/src/simple_listener/config/bridge.yaml
/opt/ros/noetic/bin/roslaunch simple_listener simple_listener.launch