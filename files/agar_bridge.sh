#!/bin/bash
echo "ROS Bridge Launch"

ROS_IP=192.168.2.240
WORKSPACE_FOLDER=/home/nikolica/workspace
HUMBLE_FOLDER=ros2_humble
BRIDGE_FOLDER=bridge_ws
BRIDGE_YAML=/home/nikolica/bridge.yaml

sleep 10s

source /opt/ros/noetic/setup.bash

export ROS_MASTER_URI=http://$ROS_IP:11311
export ROS_IP=$ROS_IP

source $WORKSPACE_FOLDER/$HUMBLE_FOLDER/install/setup.bash
source $WORKSPACE_FOLDER/$BRIDGE_FOLDER/install/setup.bash

rosparam load $BRIDGE_YAML
export ROS_HOME=$WORKSPACE_FOLDER/$HUMBLE_FOLDER
ros2 run ros1_bridge parameter_bridge

