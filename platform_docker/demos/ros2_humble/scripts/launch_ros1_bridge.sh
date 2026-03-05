#!/bin/bash

source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash

source /root/ros1_bridge_ws/install/setup.bash
/opt/ros/humble/bin/ros2 run ros1_bridge parameter_bridge