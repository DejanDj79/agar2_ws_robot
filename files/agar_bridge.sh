#!/bin/bash
set -euo pipefail

echo "AgAR ROS1<->ROS2 bridge launch"

ROS1_MASTER_IP="${ROS1_MASTER_IP:-192.168.0.61}"
ROS1_MASTER_PORT="${ROS1_MASTER_PORT:-11311}"
WORKSPACE_FOLDER="${WORKSPACE_FOLDER:-/home/nvidia/workspace}"
ROS2_WS="${ROS2_WS:-$WORKSPACE_FOLDER/agar2_ws}"
BRIDGE_WS="${BRIDGE_WS:-$WORKSPACE_FOLDER/bridge_ws}"
BRIDGE_YAML="${BRIDGE_YAML:-$ROS2_WS/files/bridge.yaml}"
ROS_HOME="${ROS_HOME:-$WORKSPACE_FOLDER/ros_bridge_home}"

detect_local_ip() {
    python3 - <<'PY'
import os
import socket

target = os.environ.get("ROS1_MASTER_IP", "192.168.0.61")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.connect((target, 11311))
    print(sock.getsockname()[0])
finally:
    sock.close()
PY
}

LOCAL_ROS_IP="${LOCAL_ROS_IP:-$(detect_local_ip)}"

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="http://${ROS1_MASTER_IP}:${ROS1_MASTER_PORT}"
export ROS_IP="$LOCAL_ROS_IP"
export ROS_HOME

echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_IP=$ROS_IP"
echo "BRIDGE_YAML=$BRIDGE_YAML"

until rosnode list >/dev/null 2>&1; do
    echo "Waiting for ROS1 master at $ROS_MASTER_URI ..."
    sleep 2
done

source /opt/ros/humble/setup.bash
if [ -f "$ROS2_WS/install/setup.bash" ]; then
    source "$ROS2_WS/install/setup.bash"
fi
if [ -f "$BRIDGE_WS/install/setup.bash" ]; then
    source "$BRIDGE_WS/install/setup.bash"
fi

if [ ! -f "$BRIDGE_YAML" ]; then
    echo "Bridge YAML not found: $BRIDGE_YAML" >&2
    exit 1
fi

rosparam load "$BRIDGE_YAML"
exec ros2 run ros1_bridge parameter_bridge