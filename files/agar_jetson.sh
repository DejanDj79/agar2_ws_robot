#!/bin/bash
echo "AgAR Jetson Launch"

sleep 1m

source /opt/ros/humble/setup.bash

cd /home/nvidia/workspace/agar2_ws
source install/setup.bash

ros2 launch agar_bringup agar_jetson_launch.py
