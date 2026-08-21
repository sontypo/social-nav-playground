#!/usr/bin/env bash
# ==============================================================================
# One-Click Launcher for Gazebo 11 / Ignition Simulation with ROS2 Social Navigation
# ==============================================================================

set -e

WORLD_FILE="${1:-social_nav_scene.world}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================================================"
echo "🤖 Social Navigation Gazebo + ROS2 Full Simulation Suite"
echo "========================================================================"
echo "World File : $WORLD_FILE"
echo "Directory  : $SCRIPT_DIR"
echo "========================================================================"

# Check ROS2 Installation
if ! command -v ros2 &> /dev/null; then
    echo "⚠️ ROS2 command not found. Sourcing /opt/ros/humble/setup.bash if available..."
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
fi

# Step 1: Launch Gazebo in background or foreground
if [[ "$WORLD_FILE" == *.sdf ]]; then
    echo "🚀 Starting Ignition Gazebo / Gazebo Sim..."
    ign gazebo "$WORLD_FILE" &
    GAZEBO_PID=$!
else
    echo "🚀 Starting Gazebo Classic 11..."
    gazebo --verbose "$WORLD_FILE" &
    GAZEBO_PID=$!
fi

sleep 3

# Step 2: Start Human Crowd Controller Node (Publishes /tracked_humans & animates peds)
echo "🚶 Launching Gazebo Human Pedestrian Controller (/tracked_humans)..."
python3 "$SCRIPT_DIR/gazebo_human_controller.py" --humans 4 --rate 20 &
HUMANS_PID=$!

# Step 3: Start Social Robot Autonomous Controller Node (Listens to /goal_pose, drives /cmd_vel)
echo "🤖 Launching Social Robot Controller (/cmd_vel)..."
python3 "$SCRIPT_DIR/social_robot_controller.py" &
ROBOT_PID=$!

# Step 4: Optional RViz2 Visualizer
if [ "$2" == "--rviz" ]; then
    echo "📊 Launching RViz2 Visualizer..."
    rviz2 -d "$SCRIPT_DIR/social_nav.rviz" &
    RVIZ_PID=$!
fi

echo "========================================================================"
echo "✅ All Social Navigation Simulation nodes are ACTIVE:"
echo "   • Gazebo PID  : $GAZEBO_PID"
echo "   • Humans PID  : $HUMANS_PID"
echo "   • Robot PID   : $ROBOT_PID"
echo "========================================================================"
echo "Press [Ctrl+C] to stop all simulation processes."

cleanup() {
    echo ""
    echo "🛑 Shutting down Gazebo simulation suite..."
    kill $HUMANS_PID $ROBOT_PID 2>/dev/null || true
    if [ -n "$RVIZ_PID" ]; then kill $RVIZ_PID 2>/dev/null || true; fi
    kill $GAZEBO_PID 2>/dev/null || true
    echo "Done."
    exit 0
}

trap cleanup SIGINT SIGTERM

wait $GAZEBO_PID
