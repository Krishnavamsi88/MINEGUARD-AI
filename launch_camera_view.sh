#!/bin/bash
# ==============================================================================
# SIH Mine Safety — HEMM Forward Driver Camera Live Viewer
# ==============================================================================
# Opens the real-time driver view from the HEMM cabin (/camera/image_raw).
# Can use the custom OpenCV HUD viewer or rqt_image_view.
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash

echo "=========================================================="
echo " Launching HEMM Driver Camera View..."
echo " Topic: /camera/image_raw (640x480 RGB, ~90° HFOV)"
echo " Press 'q' or ESC in GUI window to exit."
echo " Or run: ros2 run rqt_image_view rqt_image_view /camera/image_raw"
echo "=========================================================="

python3 "$SCRIPT_DIR/view_driver_camera.py"
