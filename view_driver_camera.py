#!/usr/bin/env python3
"""
SIH Mine Safety — HEMM Driver Forward Camera Viewer
Displays the forward-facing cabin RGB camera (/camera/image_raw).
"""

import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class DriverCameraViewer(Node):
    def __init__(self):
        super().__init__('driver_camera_viewer')
        self.bridge = CvBridge()
        self.risk_level = "UNKNOWN"
        self.frame_count = 0
        self.has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))

        self.sub_cam = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.sub_risk = self.create_subscription(
            String,
            '/safety/risk_level',
            self.risk_callback,
            10
        )

        self.get_logger().info("Driver Camera Viewer started. Waiting for /camera/image_raw...")
        if not self.has_display:
            self.get_logger().warn("No graphical display detected ($DISPLAY empty). Running in console stats mode.")

    def risk_callback(self, msg):
        self.risk_level = msg.data.strip()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        self.frame_count += 1
        h, w, _ = cv_image.shape

        # HUD Overlay
        # Header banner
        cv2.rectangle(cv_image, (0, 0), (w, 35), (20, 20, 20), -1)
        cv2.putText(cv_image, "HEMM DUMPER 01 - FORWARD CABIN VIEW", (12, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2, cv2.LINE_AA)

        # Status box
        color = (0, 255, 0)
        if "CRITICAL" in self.risk_level or "COLLISION" in self.risk_level:
            color = (0, 0, 255)
        elif "WARNING" in self.risk_level:
            color = (0, 165, 255)

        hud_text = f"RISK: {self.risk_level} | FRAMES: {self.frame_count}"
        cv2.putText(cv_image, hud_text, (w - 300, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)

        # Driving guide reticle / crosshair
        cx, cy = w // 2, int(h * 0.65)
        cv2.line(cv_image, (cx - 20, cy), (cx + 20, cy), (0, 255, 255), 1)
        cv2.line(cv_image, (cx, cy - 15), (cx, cy + 15), (0, 255, 255), 1)

        # Lane guidance markers
        cv2.line(cv_image, (int(w * 0.25), h - 10), (cx - 40, cy + 40), (0, 200, 200), 2)
        cv2.line(cv_image, (int(w * 0.75), h - 10), (cx + 40, cy + 40), (0, 200, 200), 2)

        if self.has_display:
            cv2.imshow("HEMM Driver Camera View", cv_image)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                self.get_logger().info("User quit viewer window.")
                rclpy.shutdown()
                sys.exit(0)
        else:
            if self.frame_count % 30 == 0:
                self.get_logger().info(f"[Driver Camera HUD] Received {self.frame_count} frames ({w}x{h}) | Risk: {self.risk_level}")


def main(args=None):
    rclpy.init(args=args)
    viewer = DriverCameraViewer()
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
