#!/bin/bash
echo "AgAR WS Bridge Launch"

ROS_FOLDER=/home/nikolica/workspace/agar_ws_bridge
ROS_IP=192.168.2.240
ROS_BRIDGE_SH=/home/nikolica/./start_agar_bridge.sh

echo $ROS_FOLDER

while true
do
    if ping -c 1 -W 5 $ROS_IP 1>/dev/null 2>&1 
    then
        echo "Connected!"
        break
    else
        echo "Not Connected!"
        sleep 1
    fi
done




source /opt/ros/noetic/setup.bash

export ROS_MASTER_URI=http://$ROS_IP:11311
export ROS_IP=$ROS_IP

cd $ROS_FOLDER
source devel/setup.bash
$ROS_BRIDGE_SH
roslaunch agar_bringup agar_plc.launch
