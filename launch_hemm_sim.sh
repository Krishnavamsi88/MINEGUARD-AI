#!/bin/bash
# SIH Mine Safety - HEMM Dumper Simulation Launcher
# ===================================================
# Launches Gazebo Harmonic + ROS 2 Jazzy bridge
# Usage:  ./launch_hemm_sim.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_PATH="$SCRIPT_DIR/gazebo/models"
WORLD_FILE="$SCRIPT_DIR/gazebo/worlds/sih_mine_test.sdf"

echo "=== SIH HEMM Mine Safety Simulation ==="
echo "World:      $WORLD_FILE"
echo "Model path: $MODEL_PATH"
echo ""

# Clean up any leftover processes from previous sessions
echo "Cleaning up any stale Gazebo/ROS 2 processes..."
pkill -f "gz sim"           2>/dev/null
pkill -f "ros_gz_bridge"    2>/dev/null
pkill -f "parameter_bridge" 2>/dev/null
pkill -f "teleop_twist"     2>/dev/null
sleep 1
echo "Clean."
echo ""

source /opt/ros/jazzy/setup.bash

# Make model path discoverable by Gazebo
export GZ_SIM_RESOURCE_PATH="$MODEL_PATH:$GZ_SIM_RESOURCE_PATH"

echo "=========================================================="
echo " When Gazebo opens:"
echo "   1. Click the PLAY (▶) button at the bottom-left to start simulation."
echo "   2. Notice the HEMM Dumper rests level and stable on all four wheels."
echo ""
echo " Open Terminal 2 for the ROS ↔ Gazebo bridge:"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   ros2 run ros_gz_bridge parameter_bridge \\"
echo "     /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \\"
echo "     /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"
echo ""
echo " Open Terminal 3 for keyboard teleop:"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo " Or test from CLI with a single ROS 2 command:"
echo "   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'"
echo "=========================================================="
echo ""

gz sim "$WORLD_FILE"
