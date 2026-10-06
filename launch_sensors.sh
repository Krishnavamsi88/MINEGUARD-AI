#!/bin/bash
# ==============================================================================
# SIH Mine Safety — Sensor Stack & ROS 2 Bridge Launcher
# ==============================================================================
# Starts the bidirectional ros_gz_bridge between Gazebo Harmonic and ROS 2 Jazzy
# for vehicle control, odometry, and all integrated mining sensors:
#
#   1. /cmd_vel          (ROS 2 -> Gazebo Twist drive commands)
#   2. /odom             (Gazebo -> ROS 2 Odometry state)
#   3. /scan             (Gazebo -> ROS 2 360° LiDAR scan)
#   4. /imu              (Gazebo -> ROS 2 IMU orientation & acceleration)
#   5. /gps              (Gazebo -> ROS 2 NavSatFix coordinates)
#   6. /camera/image_raw (Gazebo -> ROS 2 Front RGB Camera frame)
#   7. /camera/camera_info (Gazebo -> ROS 2 Camera calibration info)
#   8. /depth/image_raw  (Gazebo -> ROS 2 Front Depth Camera frame)
# ==============================================================================

source /opt/ros/jazzy/setup.bash

echo "=========================================================="
echo " Starting ROS 2 <-> Gazebo Sim Sensor & Control Bridge..."
echo "=========================================================="
echo " Active topics bridged:"
echo "   [IN]  /cmd_vel          : geometry_msgs/msg/Twist"
echo "   [OUT] /odom             : nav_msgs/msg/Odometry"
echo "   [OUT] /scan             : sensor_msgs/msg/LaserScan"
echo "   [OUT] /imu              : sensor_msgs/msg/Imu"
echo "   [OUT] /gps              : sensor_msgs/msg/NavSatFix"
echo "   [OUT] /camera/image_raw : sensor_msgs/msg/Image"
echo "   [OUT] /camera/camera_info: sensor_msgs/msg/CameraInfo"
echo "   [OUT] /depth/image_raw  : sensor_msgs/msg/Image"
echo "=========================================================="

ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
  /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image
