#!/usr/bin/env bash
# ==============================================================================
# SOCIALNAV STUDIO — LIVE ROBOT TELEMETRY & HARDWARE STREAM DEMO LAUNCHER
# ==============================================================================

PORT=${1:-9091}

echo "========================================================================"
echo "🚀 [SOCIALNAV STUDIO] LAUNCHING LIVE STREAM ROS2 DEMO TELEMETRY (PORT $PORT)..."
echo "========================================================================"

# 1. Source ROS2 environment (Humble / Iron / Rolling)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ Sourced ROS2 Humble"
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    source /opt/ros/iron/setup.bash
    echo "✓ Sourced ROS2 Iron"
elif [ -f "/opt/ros/rolling/setup.bash" ]; then
    source /opt/ros/rolling/setup.bash
    echo "✓ Sourced ROS2 Rolling"
else
    echo "⚠️ ROS2 setup.bash not found in standard paths (/opt/ros/*)."
    echo "Make sure ROS2 is installed and sourced in your terminal."
fi

# 2. Check and launch rosbridge WebSocket server on dedicated port
echo "🔌 Starting ROSBridge WebSocket server on ws://0.0.0.0:$PORT..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=$PORT &
BRIDGE_PID=$!

# Trap exit to cleanup background processes
cleanup() {
    echo "🛑 Stopping Live Stream Demo processes..."
    kill $BRIDGE_PID 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

sleep 2

# 3. Launch Live Robot Stream Demo Publisher
echo "📡 Starting Live Robot Stream Publisher Node (Publishing /odom, /scan, /tracked_humans, /battery_state, /imu/data)..."
python3 live_stream_publisher.py
