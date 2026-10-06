#!/bin/bash
# ==============================================================================
# SIH Mine Safety — THREE HEMM AUTONOMOUS SIMULATION
# ==============================================================================
# Single command to launch the complete 3-HEMM autonomous mine demonstration.
#
# Usage:  ./launch_3_hemm_autonomous.sh
#
# What this starts:
#   1. Gazebo Harmonic with sih_mine_world_3hemm.sdf (3 HEMM instances)
#   2. ROS 2 ↔ Gazebo bridges (3 separate bridges, one per HEMM)
#   3. Safety systems (3 instances: hemm_01, hemm_02, hemm_03)
#   4. Autonomous controllers (3 instances: hemm_01, hemm_02, hemm_03)
#
# Vehicle namespaces:
#   /hemm_01/cmd_vel  /hemm_01/odom  /hemm_01/scan  /hemm_01/safety/risk_level ...
#   /hemm_02/cmd_vel  /hemm_02/odom  /hemm_02/scan  /hemm_02/safety/risk_level ...
#   /hemm_03/cmd_vel  /hemm_03/odom  /hemm_03/scan  /hemm_03/safety/risk_level ...
#
# Spawn positions:
#   HEMM_01: x=-60, y=87.5, z=0   (Bench 1 West / Spawn Marker)  [YELLOW]
#   HEMM_02: x=+55, y=87.5, z=0   (Bench 1 East)                 [BLUE]
#   HEMM_03: x=+5,  y=87.5, z=0   (Bench 1 Mid-East)             [GREEN]
# ==============================================================================

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_PATH="$SCRIPT_DIR/gazebo/models"
WORLD_FILE="$SCRIPT_DIR/gazebo/worlds/sih_mine_world_3hemm.sdf"

# Verify world file exists
if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR: World file not found: $WORLD_FILE"
    echo "Run the generator script first."
    exit 1
fi

source /opt/ros/jazzy/setup.bash
export GZ_CONFIG_PATH="${GZ_CONFIG_PATH:-/usr/share/gz}"
export GZ_SIM_RESOURCE_PATH="$MODEL_PATH:$GZ_SIM_RESOURCE_PATH"

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║     SIH Mine Safety — 3-HEMM AUTONOMOUS SIMULATION          ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  World:  sih_mine_world_3hemm.sdf                           ║"
echo "║  HEMMs:  HEMM_01 (Yellow) | HEMM_02 (Blue) | HEMM_03 (Green)║"
echo "║  Mode:   FULLY AUTONOMOUS (waypoint navigation)              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

echo "Cleaning up stale processes..."
pkill -f "gz sim"                       2>/dev/null || true
pkill -f "ros_gz_bridge"                2>/dev/null || true
pkill -f "parameter_bridge"             2>/dev/null || true
pkill -f "hemm_autonomous_controller"   2>/dev/null || true
pkill -f "hemm_safety_system"           2>/dev/null || true
sleep 1
echo "Clean."

# ---- STEP 1: Gazebo ----
echo ""
echo "[1/4] Starting Gazebo Harmonic with 3-HEMM mine world..."
gz sim -r -v 2 "$WORLD_FILE" &
GZ_PID=$!
echo "  Gazebo PID: $GZ_PID"
echo "  Waiting 10s for Gazebo to load models..."
sleep 10

# ---- STEP 2: ROS-Gazebo Bridges ----
echo ""
echo "[2/4] Starting ROS 2 <-> Gazebo Bridges (one per HEMM)..."

ros2 run ros_gz_bridge parameter_bridge \
  /hemm_01/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /hemm_01/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /hemm_01/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /hemm_01/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /hemm_01/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
  /hemm_01/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /hemm_01/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /hemm_01/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  &> /tmp/bridge_hemm01.log &
BRIDGE1_PID=$!

ros2 run ros_gz_bridge parameter_bridge \
  /hemm_02/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /hemm_02/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /hemm_02/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /hemm_02/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /hemm_02/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
  /hemm_02/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /hemm_02/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /hemm_02/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  &> /tmp/bridge_hemm02.log &
BRIDGE2_PID=$!

ros2 run ros_gz_bridge parameter_bridge \
  /hemm_03/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /hemm_03/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /hemm_03/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /hemm_03/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /hemm_03/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
  /hemm_03/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /hemm_03/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /hemm_03/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  &> /tmp/bridge_hemm03.log &
BRIDGE3_PID=$!

echo "  Bridge PIDs: $BRIDGE1_PID, $BRIDGE2_PID, $BRIDGE3_PID"
echo "  Waiting 4s for bridges to register topics..."
sleep 4

# ---- STEP 3: Safety Systems ----
echo ""
echo "[3/4] Starting Safety Systems (3 instances)..."

python3 "$SCRIPT_DIR/hemm_safety_system.py" --vehicle-id hemm_01 \
  &> /tmp/safety_hemm01.log &
SAFE1_PID=$!

python3 "$SCRIPT_DIR/hemm_safety_system.py" --vehicle-id hemm_02 \
  &> /tmp/safety_hemm02.log &
SAFE2_PID=$!

python3 "$SCRIPT_DIR/hemm_safety_system.py" --vehicle-id hemm_03 \
  &> /tmp/safety_hemm03.log &
SAFE3_PID=$!

echo "  Safety PIDs: $SAFE1_PID, $SAFE2_PID, $SAFE3_PID"
echo "  Waiting 3s for safety systems to subscribe..."
sleep 3

# ---- STEP 4: Autonomous Controllers ----
echo ""
echo "[4/4] Starting Autonomous Controllers (3 instances)..."

python3 "$SCRIPT_DIR/hemm_autonomous_controller.py" --vehicle-id hemm_01 \
  &> /tmp/ctrl_hemm01.log &
CTRL1_PID=$!

python3 "$SCRIPT_DIR/hemm_autonomous_controller.py" --vehicle-id hemm_02 \
  &> /tmp/ctrl_hemm02.log &
CTRL2_PID=$!

python3 "$SCRIPT_DIR/hemm_autonomous_controller.py" --vehicle-id hemm_03 \
  &> /tmp/ctrl_hemm03.log &
CTRL3_PID=$!

echo "  Controller PIDs: $CTRL1_PID, $CTRL2_PID, $CTRL3_PID"

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  ALL SYSTEMS ONLINE — 3-HEMM AUTONOMOUS MINE SIMULATION     ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║                                                              ║"
echo "║  Verify topics (in another terminal):                        ║"
echo "║    ros2 topic list | grep hemm                               ║"
echo "║    ros2 topic echo /hemm_01/safety/risk_level --once         ║"
echo "║    ros2 topic echo /hemm_02/odom --once                      ║"
echo "║    ros2 topic echo /hemm_03/scan --once                      ║"
echo "║                                                              ║"
echo "║  View logs:                                                  ║"
echo "║    tail -f /tmp/safety_hemm01.log                            ║"
echo "║    tail -f /tmp/ctrl_hemm01.log                              ║"
echo "║                                                              ║"
echo "║  Press Ctrl+C to stop all processes.                         ║"
echo "╚══════════════════════════════════════════════════════════════╝"

# Trap Ctrl+C — kill all spawned processes
cleanup() {
    echo ""
    echo "Stopping all 3-HEMM simulation processes..."
    kill $CTRL1_PID $CTRL2_PID $CTRL3_PID 2>/dev/null || true
    kill $SAFE1_PID $SAFE2_PID $SAFE3_PID 2>/dev/null || true
    kill $BRIDGE1_PID $BRIDGE2_PID $BRIDGE3_PID 2>/dev/null || true
    kill $GZ_PID 2>/dev/null || true
    echo "All stopped."
}
trap cleanup EXIT INT TERM

# Keep script alive
wait $GZ_PID
