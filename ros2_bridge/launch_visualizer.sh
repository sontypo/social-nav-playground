#!/usr/bin/env bash
# 🚀 One-Command Launcher for RViz2 Visualizer + TF Broadcaster

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================================="
echo "🎯 Launching Social Navigation RViz2 Visualizer..."
echo "=========================================================="

# Source ROS2 Humble environment
source /opt/ros/humble/setup.bash

# Run python TF broadcaster in background
python3 "$DIR/social_subscriber_example.py" &
NODE_PID=$!

# Trap Ctrl+C to kill node when closing
trap "kill $NODE_PID 2>/dev/null; exit" SIGINT SIGTERM EXIT

# Launch RViz2 with pre-configured profile
rviz2 -d "$DIR/social_nav.rviz"
