#!/usr/bin/env bash
# Quick Launcher for ROS2 Rosbridge WebSocket Server

echo "=========================================================="
echo "🚀 Launching ROS2 rosbridge_server on ws://0.0.0.0:9090..."
echo "=========================================================="

# Check if rosbridge_server is installed
if ! ros2 pkg list | grep -q "rosbridge_server"; then
    echo "⚠️ rosbridge_server is not installed. Installing..."
    echo "Run: sudo apt install ros-$ROS_DISTRO-rosbridge-server"
fi

# Launch WebSocket Bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
