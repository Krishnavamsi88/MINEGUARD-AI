#!/bin/bash
# ==============================================================================
# SIH Mine Safety — HEMM Fleet Monitoring Dashboard Launcher
# ==============================================================================
# Starts the custom Python + ROS 2 web dashboard for the 3-HEMM fleet.
# Access URL: http://localhost:8080
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2 Jazzy environment
source /opt/ros/jazzy/setup.bash

echo "=========================================================="
echo " Starting HEMM Fleet Monitoring Dashboard Server..."
echo "=========================================================="
echo " Dashboard URL: http://localhost:8080"
echo " Monitoring Topics for /hemm_01, /hemm_02, /hemm_03:"
echo "   - Odometry & Kinematics"
echo "   - GPS & IMU"
echo "   - LiDAR Obstacle Detection"
echo "   - Multi-Level Collision Warning & Safety System"
echo "   - Driver Forward Cabin Cameras"
echo "   - Operator Speed Dispatch Control"
echo "=========================================================="

python3 "$SCRIPT_DIR/dashboard_server.py"
