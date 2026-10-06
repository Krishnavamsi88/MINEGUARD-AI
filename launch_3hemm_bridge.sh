#!/bin/bash
# ==============================================================================
# SIH Mine Safety — 3-HEMM ROS 2 ↔ Gazebo Bridge
# ==============================================================================
# Starts three ros_gz_bridge instances — one per HEMM namespace.
# Each instance bridges the 8 sensor/control topics for that vehicle.
#
# Namespace mapping:
#   HEMM_Dumper_01 ↔ /hemm_01/...
#   HEMM_Dumper_02 ↔ /hemm_02/...
#   HEMM_Dumper_03 ↔ /hemm_03/...
# ==============================================================================

source /opt/ros/jazzy/setup.bash

echo "=========================================================="
echo " Starting 3-HEMM ROS 2 <-> Gazebo Sensor & Control Bridges"
echo "=========================================================="

# ---- HEMM_01 Bridge ----
echo "[hemm_01] Starting bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  /hemm_01/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /hemm_01/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /hemm_01/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /hemm_01/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /hemm_01/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
  /hemm_01/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /hemm_01/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /hemm_01/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  &
BRIDGE1_PID=$!
sleep 1

# ---- HEMM_02 Bridge ----
echo "[hemm_02] Starting bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  /hemm_02/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /hemm_02/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /hemm_02/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /hemm_02/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /hemm_02/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
  /hemm_02/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /hemm_02/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /hemm_02/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  &
BRIDGE2_PID=$!
sleep 1

# ---- HEMM_03 Bridge ----
echo "[hemm_03] Starting bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  /hemm_03/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /hemm_03/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /hemm_03/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /hemm_03/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /hemm_03/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
  /hemm_03/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /hemm_03/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /hemm_03/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  &
BRIDGE3_PID=$!

echo ""
echo "  Bridge PIDs: hemm_01=$BRIDGE1_PID, hemm_02=$BRIDGE2_PID, hemm_03=$BRIDGE3_PID"
echo "  All 3 bridges running. Ctrl+C to stop all."
echo ""

# Wait for all bridges (keep script alive)
wait $BRIDGE1_PID $BRIDGE2_PID $BRIDGE3_PID
