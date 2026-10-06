#!/bin/bash
# SIH Mine Safety – HEMM in Full Mine Environment Launcher
# =========================================================
# Launches Gazebo Harmonic with the SIH open-cast mine world
# (converted from sih_mine_environment.py geometry).
#
# Usage:  ./launch_mine_world.sh
#
# This launches the mine world with the HEMM spawned on the
# spawn apron at Spawn Marker 01 (x=-60, y=87.5, z=0).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_PATH="$SCRIPT_DIR/gazebo/models"
WORLD_FILE="$SCRIPT_DIR/gazebo/worlds/sih_mine_world.sdf"

echo "=== SIH Mine Safety – Full Mine Environment ==="
echo "World:      $WORLD_FILE"
echo "Model path: $MODEL_PATH"
echo ""

echo "Cleaning up stale processes..."
pkill -f "gz sim"           2>/dev/null
pkill -f "ros_gz_bridge"    2>/dev/null
pkill -f "parameter_bridge" 2>/dev/null
pkill -f "teleop_twist"     2>/dev/null
sleep 1
echo "Clean."
echo ""

source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="$MODEL_PATH:$GZ_SIM_RESOURCE_PATH"

echo "=========================================================="
echo " MINE LAYOUT (all in Gazebo world frame):"
echo ""
echo "  Bench 1 (top, z=0):    190m x 15m ring strips"
echo "    Spawn Apron:          x=[-85,+85], y=[81,95]"
echo "    HEMM_Dumper_01 spawn: x=-60, y=87.5  (Spawn Marker 01)"
echo "    Dumping zone:         x=+30, y=87.5"
echo ""
echo "  Ramp 1 (z=0 -> z=-7):  y=[80,54], 26m run, 7m drop"
echo ""
echo "  Bench 2 (z=-7):        160m x 32m ring strips"
echo "    Connector road:       y=[54,48]"
echo ""
echo "  Ramp 2 (z=-7 -> z=-14): y=[48,20], 28m run, 7m drop"
echo ""
echo "  Mine Floor / Bench 3 (z=-14):"
echo "    Loading Zone:         x=0, y=0  (30x30m)"
echo "    Road from ramp exit:  y=[20,0]"
echo ""
echo " Open Terminal 2 – ROS 2 Sensor & Control Bridge:"
echo "   ./launch_sensors.sh"
echo ""
echo " Open Terminal 3 – Driver Assistance & Warning HUD:"
echo "   ./launch_driver_assistance.sh"
echo ""
echo " Open Terminal 4 – Human Operator Keyboard Teleop:"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo " Open Terminal 5 – Driver Forward Camera Live View:"
echo "   ./launch_camera_view.sh"
echo "   (or: ros2 run rqt_image_view rqt_image_view /camera/image_raw)"
echo "=========================================================="
echo ""

gz sim "$WORLD_FILE"
