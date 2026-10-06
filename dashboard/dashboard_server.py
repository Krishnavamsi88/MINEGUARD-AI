#!/usr/bin/env python3
"""
SIH Mine Safety — Professional Mining Fleet Control Room Dashboard Server
========================================================================
High-performance, single-process Python HTTP server and ROS 2 Jazzy node.
Features:
  - 3-HEMM real-time telemetry (Odom, GPS, IMU, LiDAR, Safety, Nav)
  - V2V Fleet Coordination monitoring (/fleet/vehicle_states)
  - Simulated Thermal Camera (/thermal/image_raw) via OpenCV Inferno colormap
  - Interactive operator target speed controls (0-20 km/h)
  - Central 2D high-resolution Gazebo Mine Map with benches, ramps & V2V vectors
  - Light-themed, modern industrial mining control-room UI

Usage:
  python3 dashboard_server.py
"""

import sys
import os
import json
import time
import math
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu, LaserScan, Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

# Spawn poses from SDF
SPAWN_POSES = {
    "hemm_01": (-60.0, 85.0, 0.0, 0.0),
    "hemm_02": ( 55.0, 91.5, 0.0, math.pi),
    "hemm_03": (  5.0, 91.5, 0.0, math.pi),
}

# Real Mine Layout Geometries from Gazebo SDF
MINE_GEOMETRY = {
    "benches": [
        {"name": "Bench 1 (Top Level 0.0m)", "z": 0.0, "x": [-100, 100], "y": [60, 105], "type": "bench1"},
        {"name": "Bench 2 (Mid Level -7.15m)", "z": -7.15, "x": [-85, 85], "y": [45, 60], "type": "bench2"},
        {"name": "Bench 3 (Floor Level -14.15m)", "z": -14.15, "x": [-50, 50], "y": [-30, 45], "type": "bench3"},
    ],
    "roads": [
        {"name": "Haul Apron (Bench 1)", "x": 0, "y": 87.5, "w": 180, "h": 18, "type": "road"},
        {"name": "Ramp 1 (Bench 1 -> 2)", "x": 0, "y": 67.0, "w": 16, "h": 27, "type": "ramp"},
        {"name": "Level 2 Connector", "x": 0, "y": 51.0, "w": 16, "h": 6, "type": "road"},
        {"name": "Ramp 2 (Bench 2 -> 3)", "x": 0, "y": 34.0, "w": 16, "h": 29, "type": "ramp"},
        {"name": "Pit Floor Access Road", "x": 0, "y": 12.0, "w": 16, "h": 18, "type": "road"},
        {"name": "Loading Turning Pad", "x": 0, "y": 2.0, "w": 34, "h": 34, "type": "zone_loading"},
        {"name": "Dumping Zone Pad", "x": 30, "y": 87.5, "w": 26, "h": 18, "type": "zone_dumping"},
    ],
    "zones": [
        {"name": "Loading Zone Face", "x": 0, "y": 2.0, "radius": 16, "color": "#2563EB", "desc": "Material Extraction Pad"},
        {"name": "Dumping Stockpile", "x": 30, "y": 87.5, "radius": 12, "color": "#D97706", "desc": "Waste & Stockpile Dump"},
        {"name": "Spawn West Apron", "x": -60, "y": 87.5, "radius": 8, "color": "#4B5563", "desc": "HEMM Staging Apron"},
    ],
    "conflict_zones": [
        {"name": "Ramp 1 Junction", "x": 0.0, "y": 84.0, "radius": 11.0, "code": "ZONE_RAMP1"},
        {"name": "Pit Loading Zone", "x": 0.0, "y": 3.0, "radius": 9.0, "code": "ZONE_LOADING"},
        {"name": "Dumping Zone", "x": 30.0, "y": 88.0, "radius": 9.0, "code": "ZONE_DUMPING"},
        {"name": "Ramp 2 Junction", "x": 0.0, "y": 51.0, "radius": 7.0, "code": "ZONE_RAMP2"},
    ],
    "bounds": {
        "x_min": -110, "x_max": 110,
        "y_min": -40, "y_max": 115
    }
}


class DashboardROSNode(Node):
    def __init__(self):
        super().__init__('hemm_dashboard_backend')
        self.bridge = CvBridge()

        # State storage for all 3 HEMMs
        self.state = {}
        for vid in ["hemm_01", "hemm_02", "hemm_03"]:
            sx, sy, sz, syaw = SPAWN_POSES[vid]
            self.state[vid] = {
                "id": vid,
                "label": f"HEMM-{vid.split('_')[1]}",
                "color": "#EAB308" if vid == "hemm_01" else ("#2563EB" if vid == "hemm_02" else "#16A34A"),
                "status": "AUTONOMOUS",
                "autonomous_status": "ACTIVE",
                "cycle_phase": "HAULING_TO_LOAD" if vid != "hemm_02" else "DUMPING",
                "v2v_state": "NORMAL",
                "leader_id": None,
                "dist_to_leader_m": None,
                "lateral_offset_m": 0.0,
                "route": "BENCH 1 <-> RAMPS <-> LOADING ZONE",
                "next_wp": [0.0, 87.5, 0.0],
                "wp_index": 0,
                "total_wps": 20,
                "dist_to_wp": 0.0,
                "x": sx,
                "y": sy,
                "z": sz,
                "yaw": syaw,
                "heading_deg": round(math.degrees(syaw) % 360, 1),
                "speed_mps": 0.0,
                "speed_kmh": 0.0,
                "target_speed_mps": 2.2,
                "target_speed_kmh": 8.0,
                "gps": {"lat": 21.9167, "lon": 85.3333, "alt": 450.0},
                "imu": {"pitch": 0.0, "roll": 0.0, "yaw": 0.0},
                "lidar": {"nearest_obstacle_dist": 50.0, "obstacle_detected": False},
                "safety": {
                    "risk_level": "CLEAR",
                    "risk_code": 0,
                    "advisory": "HAUL ROAD CLEAR — NORMAL AUTONOMOUS OPERATION",
                    "closest_sector": "front",
                    "obstacle_dist": 50.0,
                    "ttc_s": None
                },
                "sensors": {
                    "gps": False,
                    "lidar": False,
                    "imu": False,
                    "camera": False,
                    "thermal": False,
                    "odom": False
                },
                "last_update": time.time()
            }

        # Latest JPEG image buffers for camera streaming
        self.latest_rgb_images = {"hemm_01": None, "hemm_02": None, "hemm_03": None}
        self.latest_thermal_images = {"hemm_01": None, "hemm_02": None, "hemm_03": None}

        # Event log buffer (circular deque)
        self.events = deque(maxlen=60)
        self._log_event("SYSTEM", "INITIALIZATION", "Mining Fleet Dispatch Control Room initialized.", "INFO")

        # Speed command publishers
        self.speed_publishers = {
            "hemm_01": self.create_publisher(Float32, '/hemm_01/set_target_speed', 10),
            "hemm_02": self.create_publisher(Float32, '/hemm_02/set_target_speed', 10),
            "hemm_03": self.create_publisher(Float32, '/hemm_03/set_target_speed', 10),
        }

        # Thermal Image Publishers (ROS 2 topics for ecosystem compatibility)
        self.thermal_publishers = {
            "hemm_01": self.create_publisher(Image, '/hemm_01/thermal/image_raw', 10),
            "hemm_02": self.create_publisher(Image, '/hemm_02/thermal/image_raw', 10),
            "hemm_03": self.create_publisher(Image, '/hemm_03/thermal/image_raw', 10),
        }

        # V2V Shared Topic Subscriber
        self.create_subscription(String, '/fleet/vehicle_states', self._cb_fleet_v2v, 10)

        # Vehicle Subscriptions
        for vid in ["hemm_01", "hemm_02", "hemm_03"]:
            self._setup_subscribers(vid)

        self.get_logger().info("Dashboard ROS Node initialized with V2V and Thermal streaming.")

    def _log_event(self, vid: str, category: str, message: str, severity: str = "INFO"):
        self.events.appendleft({
            "time": time.strftime("%H:%M:%S"),
            "vehicle": vid,
            "category": category,
            "message": message,
            "severity": severity
        })

    def _setup_subscribers(self, vid: str):
        self.create_subscription(Odometry, f'/{vid}/odom', lambda msg, v=vid: self._cb_odom(v, msg), 10)
        self.create_subscription(NavSatFix, f'/{vid}/gps', lambda msg, v=vid: self._cb_gps(v, msg), 10)
        self.create_subscription(Imu, f'/{vid}/imu', lambda msg, v=vid: self._cb_imu(v, msg), 10)
        self.create_subscription(LaserScan, f'/{vid}/scan', lambda msg, v=vid: self._cb_scan(v, msg), 10)
        self.create_subscription(String, f'/{vid}/safety/warning', lambda msg, v=vid: self._cb_safety_warning(v, msg), 10)
        self.create_subscription(Float32, f'/{vid}/target_speed', lambda msg, v=vid: self._cb_target_speed(v, msg), 10)
        self.create_subscription(String, f'/{vid}/nav_status', lambda msg, v=vid: self._cb_nav_status(v, msg), 10)
        self.create_subscription(Image, f'/{vid}/camera/image_raw', lambda msg, v=vid: self._cb_camera(v, msg), 10)
        self.create_subscription(String, f'/{vid}/v2v/state', lambda msg, v=vid: self._cb_vehicle_v2v_state(v, msg), 10)

    def _cb_vehicle_v2v_state(self, vid: str, msg: String):
        val = msg.data.strip()
        if vid in self.state:
            old_s = self.state[vid].get("v2v_state")
            self.state[vid]["v2v_state"] = val
            if old_s != val:
                sev = "WARNING" if "WAITING" in val else ("SUCCESS" if ("GO" in val or "CROSSING" in val or "CLEAR" in val) else "INFO")
                self._log_event(vid.upper(), "V2V_STATUS", f"V2V: {val}", sev)

    def _cb_fleet_v2v(self, msg: String):
        try:
            data = json.loads(msg.data)
            vid = data.get("vehicle_id")
            if vid in self.state:
                v = self.state[vid]
                old_state = v.get("v2v_state")
                new_state = data.get("state", "NORMAL")
                v["v2v_state"] = new_state
                v["leader_id"] = data.get("leader_id")
                v["dist_to_leader_m"] = data.get("dist_to_leader_m")
                v["lateral_offset_m"] = data.get("lateral_offset_m", 0.0)
                v["conflict_zone"] = data.get("conflict_zone")

                # Update live coordinates, heading and speed directly from controller fleet packet
                if "x" in data and data["x"] is not None:
                    v["x"] = float(data["x"])
                if "y" in data and data["y"] is not None:
                    v["y"] = float(data["y"])
                if "z" in data and data["z"] is not None:
                    v["z"] = float(data["z"])
                if "yaw" in data and data["yaw"] is not None:
                    v["yaw"] = float(data["yaw"])
                    v["heading_deg"] = round((math.degrees(float(data["yaw"])) + 360.0) % 360.0, 1)
                if "speed_kmh" in data and data["speed_kmh"] is not None:
                    v["speed_kmh"] = float(data["speed_kmh"])
                    v["speed_mps"] = round(float(data["speed_kmh"]) / 3.6, 2)
                if "phase" in data and data["phase"]:
                    v["cycle_phase"] = data["phase"]
                if "wp_index" in data and data["wp_index"] is not None:
                    v["wp_index"] = int(data["wp_index"])
                v["last_update"] = time.time()

                # Log notable V2V coordination transitions
                if old_state != new_state:
                    sev = "WARNING" if "WAITING" in new_state else ("SUCCESS" if ("GO" in new_state or "CROSSING" in new_state or "CLEAR" in new_state) else "INFO")
                    self._log_event(vid.upper(), "V2V_COORDINATION", f"{vid.upper()} -> {new_state}", sev)
        except Exception:
            pass

    def _cb_odom(self, vid: str, msg: Odometry):
        v = self.state[vid]
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        oz = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        odom_yaw = math.atan2(siny, cosy)

        sx, sy, sz, syaw = SPAWN_POSES[vid]
        cos_s, sin_s = math.cos(syaw), math.sin(syaw)

        if not v["sensors"]["gps"]:
            v["x"] = round(sx + (ox * cos_s - oy * sin_s), 2)
            v["y"] = round(sy + (ox * sin_s + oy * cos_s), 2)
            v["z"] = round(sz + oz, 2)

        if not v["sensors"]["imu"]:
            total_yaw = (odom_yaw + syaw)
            while total_yaw > math.pi: total_yaw -= 2 * math.pi
            while total_yaw < -math.pi: total_yaw += 2 * math.pi
            v["yaw"] = round(total_yaw, 3)
            v["heading_deg"] = round((math.degrees(total_yaw) + 360) % 360, 1)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        mps = math.sqrt(vx**2 + vy**2)
        v["speed_mps"] = round(mps, 2)
        v["speed_kmh"] = round(mps * 3.6, 1)
        v["sensors"]["odom"] = True
        v["last_update"] = time.time()

    def _cb_gps(self, vid: str, msg: NavSatFix):
        v = self.state[vid]
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            v["gps"]["lat"] = round(msg.latitude, 6)
            v["gps"]["lon"] = round(msg.longitude, 6)
            v["gps"]["alt"] = round(msg.altitude, 1)
            v["x"] = round((msg.longitude - 85.3333) * 103275.0, 2)
            v["y"] = round((msg.latitude - 21.9167) * 111320.0, 2)
            v["z"] = round(msg.altitude - 450.0, 2)
            v["sensors"]["gps"] = True

    def _cb_imu(self, vid: str, msg: Imu):
        v = self.state[vid]
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        sinp = 2 * (qw * qy - qz * qx)
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
        sinr = 2 * (qw * qx + qy * qz)
        cosr = 1 - 2 * (qx**2 + qy**2)
        roll = math.degrees(math.atan2(sinr, cosr))
        v["imu"]["pitch"] = round(pitch, 1)
        v["imu"]["roll"] = round(roll, 1)
        v["sensors"]["imu"] = True

    def _cb_scan(self, vid: str, msg: LaserScan):
        v = self.state[vid]
        valid_ranges = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r) and msg.range_min < r < msg.range_max]
        if valid_ranges:
            m = min(valid_ranges)
            v["lidar"]["nearest_obstacle_dist"] = round(m, 2)
            v["lidar"]["obstacle_detected"] = (m < 25.0)
        v["sensors"]["lidar"] = True

    def _cb_safety_warning(self, vid: str, msg: String):
        v = self.state[vid]
        try:
            p = json.loads(msg.data)
            old_risk = v["safety"]["risk_level"]
            new_risk = p.get("risk_level", "CLEAR")
            v["safety"]["risk_level"] = new_risk
            v["safety"]["risk_code"] = p.get("risk_code", 0)
            v["safety"]["advisory"] = p.get("driver_advisory", "")
            v["safety"]["closest_sector"] = p.get("closest_sector", "front")
            v["safety"]["obstacle_dist"] = p.get("obstacle_distance_m", 50.0)
            v["safety"]["ttc_s"] = p.get("ttc_seconds")

            if old_risk != new_risk and new_risk in ["WARNING", "CRITICAL"]:
                self._log_event(vid.upper(), "SAFETY_ALERT", f"Safety Risk Level {new_risk}: {v['safety']['advisory']}", "DANGER")
        except Exception:
            pass

    def _cb_target_speed(self, vid: str, msg: Float32):
        v = self.state[vid]
        v["target_speed_mps"] = round(float(msg.data), 2)
        v["target_speed_kmh"] = round(float(msg.data) * 3.6, 1)

    def _cb_nav_status(self, vid: str, msg: String):
        v = self.state[vid]
        try:
            p = json.loads(msg.data)
            v["status"] = p.get("status", "AUTONOMOUS")
            v["autonomous_status"] = p.get("autonomous_status", "ACTIVE")
            v["cycle_phase"] = p.get("cycle_phase", v["cycle_phase"])
            v["v2v_state"] = p.get("v2v_state", v["v2v_state"])
            v["route"] = p.get("route", v["route"])
            v["next_wp"] = p.get("target_wp", v["next_wp"])
            v["wp_index"] = p.get("wp_index", v["wp_index"])
            v["total_wps"] = p.get("total_wps", v["total_wps"])
            v["dist_to_wp"] = p.get("dist_to_wp_m", 0.0)
            if "target_speed_mps" in p:
                v["target_speed_mps"] = p["target_speed_mps"]
                v["target_speed_kmh"] = p["target_speed_kmh"]
            if "leader_id" in p:
                v["leader_id"] = p["leader_id"]
                v["dist_to_leader_m"] = p["dist_to_leader_m"]
            if "lateral_offset_m" in p:
                v["lateral_offset_m"] = p["lateral_offset_m"]
        except Exception:
            pass

    def _cb_camera(self, vid: str, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 1. Optical RGB Feed Processing
            rgb_disp = cv_img.copy()
            timestamp_str = time.strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(rgb_disp, f"{vid.upper()} CABIN OPTICAL | {timestamp_str}", (12, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
            risk_text = f"RISK: {self.state[vid]['safety']['risk_level']}"
            r_color = (0, 255, 0) if self.state[vid]['safety']['risk_level'] == "CLEAR" else (0, 0, 255)
            cv2.putText(rgb_disp, risk_text, (12, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, r_color, 2, cv2.LINE_AA)

            ret_rgb, buf_rgb = cv2.imencode('.jpg', rgb_disp, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret_rgb:
                self.latest_rgb_images[vid] = buf_rgb.tobytes()
                self.state[vid]["sensors"]["camera"] = True

            # 2. Simulated Thermal IR Processing (Inferno Colormap)
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            # Apply adaptive histogram contrast stretch to accentuate terrain and heat edges
            clahe = cv2.createCLAHE(clipLimit=2.2, tileGridSize=(8, 8))
            enhanced_gray = clahe.apply(gray)
            thermal_bgr = cv2.applyColorMap(enhanced_gray, cv2.COLORMAP_INFERNO)

            cv2.putText(thermal_bgr, f"{vid.upper()} FLIR THERMAL IR | RANGE: 28.5C - 88.0C", (12, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(thermal_bgr, f"V2V: {self.state[vid]['v2v_state']}", (12, 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)

            ret_th, buf_th = cv2.imencode('.jpg', thermal_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret_th:
                self.latest_thermal_images[vid] = buf_th.tobytes()
                self.state[vid]["sensors"]["thermal"] = True

            # Publish simulated thermal Image topic onto ROS 2
            try:
                th_msg = self.bridge.cv2_to_imgmsg(thermal_bgr, encoding='bgr8')
                th_msg.header.stamp = msg.header.stamp
                th_msg.header.frame_id = f"{vid}_camera_thermal_frame"
                self.thermal_publishers[vid].publish(th_msg)
            except Exception:
                pass

        except Exception:
            pass

    def set_vehicle_speed(self, vid: str, speed_kmh: float):
        if vid in self.speed_publishers:
            speed_mps = max(0.0, min(5.56, float(speed_kmh) / 3.6))
            msg = Float32()
            msg.data = float(speed_mps)
            self.speed_publishers[vid].publish(msg)
            self.state[vid]["target_speed_mps"] = round(speed_mps, 2)
            self.state[vid]["target_speed_kmh"] = round(speed_kmh, 1)
            self._log_event(vid.upper(), "SPEED_DISPATCH", f"Target speed adjusted to {speed_kmh:.1f} km/h ({speed_mps:.2f} m/s)", "INFO")


# Global reference to ROS node
ros_node = None

# Professional Mining Fleet Control Room Dashboard HTML
DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>MINEGUARD AI — Autonomous Mining Fleet Command</title>
  <style>
    :root {
      --bg: #F4F6F9;
      --card: #FFFFFF;
      --border: #E2E8F0;
      --border-dark: #CBD5E1;
      --text: #0F172A;
      --muted: #64748B;
      --accent: #2563EB;
      --accent-hover: #1D4ED8;
      --success: #16A34A;
      --warning: #D97706;
      --danger: #DC2626;
      --sidebar-w: 220px;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    body { background: var(--bg); color: var(--text); font-size: 13px; line-height: 1.4; height: 100vh; display: flex; overflow: hidden; }

    /* Left Sidebar */
    aside.sidebar {
      width: var(--sidebar-w);
      background: #0F172A;
      color: #F8FAFC;
      display: flex;
      flex-direction: column;
      flex-shrink: 0;
      border-right: 1px solid #1E293B;
    }
    .brand {
      padding: 16px;
      border-bottom: 1px solid #1E293B;
      display: flex;
      align-items: center;
      gap: 10px;
    }
    .brand-icon {
      background: #2563EB;
      width: 32px;
      height: 32px;
      border-radius: 6px;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 16px;
    }
    .brand-text h1 { font-size: 14px; font-weight: 800; letter-spacing: 0.02em; }
    .brand-text p { font-size: 10px; color: #94A3B8; text-transform: uppercase; font-weight: 600; }
    
    .nav-list { list-style: none; padding: 12px 8px; flex: 1; }
    .nav-item {
      padding: 10px 12px;
      border-radius: 6px;
      font-size: 12px;
      font-weight: 600;
      color: #94A3B8;
      display: flex;
      align-items: center;
      gap: 10px;
      cursor: pointer;
      margin-bottom: 4px;
      transition: all 0.15s;
    }
    .nav-item:hover { color: #FFFFFF; background: #1E293B; }
    .nav-item.active { color: #FFFFFF; background: #2563EB; }

    .sidebar-health {
      padding: 12px;
      background: #1E293B;
      border-top: 1px solid #334155;
      margin: 10px;
      border-radius: 6px;
      font-size: 11px;
    }
    .health-row { display: flex; justify-content: space-between; margin-bottom: 4px; color: #CBD5E1; }
    .health-row:last-child { margin-bottom: 0; }
    .pill-green { color: #4ADE80; font-weight: 700; }

    /* Main Container */
    main.main-content {
      flex: 1;
      display: flex;
      flex-direction: column;
      overflow: hidden;
    }

    /* Header */
    header.top-header {
      background: var(--card);
      border-bottom: 1px solid var(--border);
      padding: 10px 24px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      height: 52px;
      flex-shrink: 0;
    }
    .header-left { display: flex; align-items: center; gap: 14px; }
    .header-title { font-size: 15px; font-weight: 800; letter-spacing: -0.01em; }
    .site-pill {
      background: #F1F5F9;
      color: #475569;
      font-size: 11px;
      font-weight: 600;
      padding: 3px 8px;
      border-radius: 4px;
      border: 1px solid var(--border);
    }
    .header-right { display: flex; align-items: center; gap: 16px; font-size: 12px; }
    .live-clock { font-family: monospace; font-weight: 700; color: var(--text); background: #F8FAFC; padding: 4px 8px; border-radius: 4px; border: 1px solid var(--border); }
    .heartbeat { width: 8px; height: 8px; border-radius: 50%; background: #16A34A; display: inline-block; animation: pulse 1.5s infinite; }
    @keyframes pulse { 0% { opacity: 1; transform: scale(1); } 50% { opacity: 0.4; transform: scale(0.85); } 100% { opacity: 1; transform: scale(1); } }

    /* KPI Summary Row */
    .kpi-row {
      display: grid;
      grid-template-columns: repeat(6, 1fr);
      gap: 12px;
      padding: 12px 24px;
      background: #FFFFFF;
      border-bottom: 1px solid var(--border);
      flex-shrink: 0;
    }
    .kpi-card {
      border: 1px solid var(--border);
      border-radius: 6px;
      padding: 8px 12px;
      background: #F8FAFC;
    }
    .kpi-label { font-size: 10px; font-weight: 700; color: var(--muted); text-transform: uppercase; letter-spacing: 0.04em; margin-bottom: 2px; }
    .kpi-value { font-size: 18px; font-weight: 800; color: var(--text); }
    .kpi-sub { font-size: 10px; color: var(--muted); font-weight: 500; }

    /* Body 2-Column Split */
    .body-grid {
      flex: 1;
      display: grid;
      grid-template-columns: 1fr 430px;
      overflow: hidden;
    }

    /* Map Panel */
    .map-panel {
      position: relative;
      background: #FFFFFF;
      border-right: 1px solid var(--border);
      display: flex;
      flex-direction: column;
    }
    .map-toolbar {
      padding: 8px 16px;
      border-bottom: 1px solid var(--border);
      display: flex;
      justify-content: space-between;
      align-items: center;
      background: #F8FAFC;
    }
    .map-title { font-size: 12px; font-weight: 700; text-transform: uppercase; color: var(--muted); letter-spacing: 0.05em; }
    .map-tools { display: flex; gap: 6px; }
    .btn-tool {
      background: #FFFFFF;
      border: 1px solid var(--border-dark);
      border-radius: 4px;
      padding: 4px 8px;
      font-size: 11px;
      font-weight: 600;
      cursor: pointer;
    }
    .btn-tool:hover { background: #E2E8F0; }

    .canvas-container {
      flex: 1;
      position: relative;
      background: #FAFCFE;
      overflow: hidden;
    }
    canvas#mineMap { width: 100%; height: 100%; display: block; }

    .map-legend {
      position: absolute;
      bottom: 12px;
      left: 12px;
      background: rgba(255, 255, 255, 0.95);
      border: 1px solid var(--border);
      border-radius: 6px;
      padding: 8px 12px;
      font-size: 11px;
      box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      display: flex;
      flex-direction: column;
      gap: 4px;
    }
    .legend-row { display: flex; align-items: center; gap: 8px; }
    .leg-color { width: 10px; height: 10px; border-radius: 2px; }

    /* Control & Telemetry Panel (Right) */
    .telemetry-panel {
      background: #FFFFFF;
      display: flex;
      flex-direction: column;
      overflow-y: auto;
    }

    /* Vehicle Selection Tabs */
    .vehicle-tabs {
      display: flex;
      padding: 10px 16px;
      gap: 6px;
      border-bottom: 1px solid var(--border);
      background: #F8FAFC;
    }
    .v-tab {
      flex: 1;
      padding: 7px 4px;
      text-align: center;
      border: 1px solid var(--border);
      border-radius: 4px;
      font-weight: 700;
      font-size: 12px;
      cursor: pointer;
      background: #FFFFFF;
      transition: all 0.15s;
    }
    .v-tab:hover { background: #F1F5F9; }
    .v-tab.active { background: #0F172A; color: #FFFFFF; border-color: #0F172A; }

    .panel-section {
      padding: 12px 16px;
      border-bottom: 1px solid var(--border);
    }
    .sec-hdr {
      font-size: 11px;
      font-weight: 800;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.05em;
      margin-bottom: 10px;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    /* Camera View & Stream Toggle */
    .camera-frame {
      aspect-ratio: 16 / 9;
      background: #000000;
      border-radius: 6px;
      overflow: hidden;
      position: relative;
      border: 1px solid var(--border);
    }
    .camera-frame img { width: 100%; height: 100%; object-fit: cover; display: block; }
    .cam-tag {
      position: absolute;
      top: 6px;
      left: 6px;
      background: rgba(0, 0, 0, 0.7);
      color: #FFFFFF;
      font-size: 10px;
      font-weight: 700;
      padding: 2px 6px;
      border-radius: 3px;
    }
    .cam-modes {
      display: flex;
      gap: 6px;
      margin-top: 8px;
    }
    .cam-btn {
      flex: 1;
      padding: 6px;
      font-size: 11px;
      font-weight: 700;
      border: 1px solid var(--border-dark);
      border-radius: 4px;
      background: #FFFFFF;
      cursor: pointer;
      text-align: center;
    }
    .cam-btn.active {
      background: #2563EB;
      color: #FFFFFF;
      border-color: #2563EB;
    }

    /* Speed Control Station */
    .speed-dashboard {
      background: #F8FAFC;
      border: 1px solid var(--border);
      border-radius: 6px;
      padding: 12px;
    }
    .speed-grid {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 8px;
      text-align: center;
      margin-bottom: 12px;
    }
    .sp-box { background: #FFFFFF; border: 1px solid var(--border); border-radius: 4px; padding: 6px; }
    .sp-val { font-size: 17px; font-weight: 800; color: var(--text); }
    .sp-lbl { font-size: 9px; font-weight: 700; color: var(--muted); text-transform: uppercase; }

    .preset-btns {
      display: grid;
      grid-template-columns: repeat(5, 1fr);
      gap: 4px;
      margin-bottom: 10px;
    }
    .p-btn {
      padding: 5px 0;
      font-size: 11px;
      font-weight: 700;
      border: 1px solid var(--border-dark);
      background: #FFFFFF;
      border-radius: 4px;
      cursor: pointer;
      text-align: center;
    }
    .p-btn:hover { background: #E2E8F0; }
    .p-btn.stop { background: #FEE2E2; color: #991B1B; border-color: #FECACA; }

    .slider-row {
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .slider-row input[type="range"] { flex: 1; accent-color: #2563EB; }
    .step-btn {
      width: 28px;
      height: 28px;
      border: 1px solid var(--border-dark);
      background: #FFFFFF;
      font-weight: 800;
      border-radius: 4px;
      cursor: pointer;
    }

    /* V2V Coordination Box */
    .v2v-box {
      background: #EFF6FF;
      border: 1px solid #BFDBFE;
      border-radius: 6px;
      padding: 10px 12px;
      margin-bottom: 10px;
    }
    .v2v-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; }
    .v2v-state-tag {
      font-size: 11px;
      font-weight: 800;
      padding: 2px 8px;
      border-radius: 4px;
      text-transform: uppercase;
      background: #2563EB;
      color: #FFFFFF;
    }
    .v2v-desc { font-size: 11px; color: #1E40AF; }

    /* Telemetry Table */
    .tele-table { width: 100%; border-collapse: collapse; font-size: 12px; }
    .tele-table tr { border-bottom: 1px solid #F1F5F9; }
    .tele-table tr:last-child { border-bottom: none; }
    .tele-table td { padding: 4px 0; }
    .t-lbl { color: var(--muted); font-weight: 500; }
    .t-val { text-align: right; font-weight: 700; font-family: monospace; }

    /* Risk badges */
    .badge { padding: 2px 7px; border-radius: 3px; font-weight: 800; font-size: 10px; text-transform: uppercase; }
    .badge-CLEAR { background: #DCFCE7; color: #166534; }
    .badge-ADVISORY { background: #E0F2FE; color: #075985; }
    .badge-CAUTION { background: #FEF3C7; color: #92400E; }
    .badge-WARNING { background: #FCE7F3; color: #9D174D; }
    .badge-CRITICAL { background: #FEE2E2; color: #991B1B; }

    .sensor-list { display: flex; gap: 4px; justify-content: flex-end; }
    .sens-tag { font-size: 9px; font-weight: 700; padding: 2px 4px; border-radius: 3px; }
    .sens-on { background: #DCFCE7; color: #166534; }
    .sens-off { background: #F1F5F9; color: #94A3B8; }

    /* Bottom Event Log */
    .event-log-panel {
      height: 140px;
      border-top: 1px solid var(--border);
      background: #FFFFFF;
      overflow-y: auto;
      padding: 8px 16px;
      flex-shrink: 0;
    }
    .log-table { width: 100%; border-collapse: collapse; font-size: 11px; }
    .log-table th { text-align: left; padding: 4px 6px; color: var(--muted); border-bottom: 1px solid var(--border); }
    .log-table td { padding: 4px 6px; border-bottom: 1px solid #F8FAFC; }
    .sev-INFO { color: #2563EB; font-weight: 600; }
    .sev-WARNING { color: #D97706; font-weight: 700; }
    .sev-DANGER { color: #DC2626; font-weight: 800; }
  </style>
</head>
<body>

  <!-- Left Sidebar Navigation -->
  <aside class="sidebar">
    <div class="brand">
      <div class="brand-icon">⛏️</div>
      <div class="brand-text">
        <h1>MINEGUARD AI</h1>
        <p>FLEET COMMAND HQ</p>
      </div>
    </div>
    <ul class="nav-list">
      <li class="nav-item active">⊞ Fleet Overview</li>
      <li class="nav-item">🚚 Haul Trucks (3)</li>
      <li class="nav-item">🗺️ Pit Bench Map</li>
      <li class="nav-item">📡 V2V Coordination</li>
      <li class="nav-item">⚠️ Safety Matrix</li>
      <li class="nav-item">📹 Cabin & Thermal Cams</li>
      <li class="nav-item">📊 Haul Cycle Logs</li>
    </ul>

    <div class="sidebar-health">
      <div class="health-row"><span>ROS 2 Jazzy</span><span class="pill-green">ONLINE</span></div>
      <div class="health-row"><span>Gazebo Harmonic</span><span class="pill-green">SYNCED</span></div>
      <div class="health-row"><span>V2V Mesh Link</span><span class="pill-green">5 Hz ACTIVE</span></div>
      <div class="health-row"><span>Units Connected</span><span class="pill-green">3 / 3</span></div>
    </div>
  </aside>

  <!-- Main Content Area -->
  <main class="main-content">
    <!-- Top Header -->
    <header class="top-header">
      <div class="header-left">
        <span class="heartbeat"></span>
        <span class="header-title">OPEN-PIT AUTONOMOUS HAULAGE SYSTEM</span>
        <span class="site-pill">NORTH PIT BENCH 1 & PIT FLOOR (-14.15m to 0.0m)</span>
        <span class="site-pill">HAUL ROAD: 16–18m WIDE</span>
      </div>
      <div class="header-right">
        <span>SHIFT: <b>ALPHA-1</b></span>
        <span>OPERATOR: <b>DISPATCH-418</b></span>
        <span class="live-clock" id="liveClock">00:00:00 UTC</span>
      </div>
    </header>

    <!-- Top 6 KPI Summary Cards -->
    <div class="kpi-row">
      <div class="kpi-card">
        <div class="kpi-label">Active Haul Fleet</div>
        <div class="kpi-value" id="kpi-fleet-count">3 / 3</div>
        <div class="kpi-sub">100% Operational</div>
      </div>
      <div class="kpi-card">
        <div class="kpi-label">Fleet Mean Speed</div>
        <div class="kpi-value" id="kpi-mean-speed">0.0 <span style="font-size:12px;">km/h</span></div>
        <div class="kpi-sub" id="kpi-speed-sub">Target: 8.0 km/h</div>
      </div>
      <div class="kpi-card">
        <div class="kpi-label">Cycle Operations</div>
        <div class="kpi-value" id="kpi-cycles" style="font-size:15px; padding-top:2px;">HAULING</div>
        <div class="kpi-sub">Multi-bench loop active</div>
      </div>
      <div class="kpi-card">
        <div class="kpi-label">Fleet Safety Status</div>
        <div class="kpi-value" id="kpi-safety-status" style="color:var(--success);">ALL CLEAR</div>
        <div class="kpi-sub">0 Critical Hazards</div>
      </div>
      <div class="kpi-card">
        <div class="kpi-label">V2V Coordination</div>
        <div class="kpi-value" style="color:var(--accent);">ACTIVE</div>
        <div class="kpi-sub">Headway & Passing Mesh</div>
      </div>
      <div class="kpi-card">
        <div class="kpi-label">LiDAR Min Clearance</div>
        <div class="kpi-value" id="kpi-min-dist">50.0 <span style="font-size:12px;">m</span></div>
        <div class="kpi-sub">Forward Path Safe</div>
      </div>
    </div>

    <!-- Center 2-Column Layout -->
    <div class="body-grid">
      <!-- Left: High-Definition Mine Canvas Map -->
      <div class="map-panel">
        <div class="map-toolbar">
          <div class="map-title">Gazebo Open-Pit Mine Top-Down Telemetry Map</div>
          <div class="map-tools">
            <button class="btn-tool" onclick="resetMapZoom()">Reset View</button>
            <button class="btn-tool" onclick="zoomMap(1.2)">+ Zoom</button>
            <button class="btn-tool" onclick="zoomMap(0.8)">- Zoom</button>
          </div>
        </div>

        <div class="canvas-container">
          <canvas id="mineMap"></canvas>

          <div class="map-legend">
            <div class="legend-row"><div class="leg-color" style="background:#E2E8F0;"></div><span>Bench 1 (0.0m)</span></div>
            <div class="legend-row"><div class="leg-color" style="background:#CBD5E1;"></div><span>Bench 2 (-7.15m)</span></div>
            <div class="legend-row"><div class="leg-color" style="background:#94A3B8;"></div><span>Pit Floor (-14.15m)</span></div>
            <div class="legend-row"><div class="leg-color" style="background:#EAB308;"></div><span>HEMM-01 (Yellow)</span></div>
            <div class="legend-row"><div class="leg-color" style="background:#2563EB;"></div><span>HEMM-02 (Blue)</span></div>
            <div class="legend-row"><div class="leg-color" style="background:#16A34A;"></div><span>HEMM-03 (Green)</span></div>
            <div class="legend-row"><div class="leg-color" style="background:#818CF8;"></div><span>V2V Cooperative Link</span></div>
          </div>
        </div>
      </div>

      <!-- Right: Detailed Control & Telemetry Panel -->
      <div class="telemetry-panel">
        <!-- Vehicle Select Tabs -->
        <div class="vehicle-tabs">
          <div class="v-tab active" id="tab-hemm_01" onclick="selectVehicle('hemm_01')">HEMM-01 (Yellow)</div>
          <div class="v-tab" id="tab-hemm_02" onclick="selectVehicle('hemm_02')">HEMM-02 (Blue)</div>
          <div class="v-tab" id="tab-hemm_03" onclick="selectVehicle('hemm_03')">HEMM-03 (Green)</div>
        </div>

        <!-- Camera Section with RGB / Thermal Toggle -->
        <div class="panel-section">
          <div class="sec-hdr">
            <span>Cabin Forward Sensor Stream</span>
            <span id="cam-active-mode-label" style="color:var(--accent); font-weight:700;">OPTICAL RGB</span>
          </div>

          <div class="camera-frame">
            <img id="cameraStream" src="" alt="Connecting to Camera Stream...">
            <div class="cam-tag" id="camOverlayTag">HEMM-01 OPTICAL</div>
          </div>

          <div class="cam-modes">
            <button class="cam-btn active" id="btn-cam-rgb" onclick="setCameraMode('rgb')">📹 RGB OPTICAL</button>
            <button class="cam-btn" id="btn-cam-thermal" onclick="setCameraMode('thermal')">🌡️ THERMAL FLIR</button>
          </div>
        </div>

        <!-- Operator Target Speed Control -->
        <div class="panel-section">
          <div class="sec-hdr">
            <span>Operator Target Speed Dispatch</span>
            <span id="ctrl-reason" style="font-size:10px; color:var(--muted);">OPERATOR CRUISING SETPOINT</span>
          </div>

          <div class="speed-dashboard">
            <div class="speed-grid">
              <div class="sp-box">
                <div class="sp-val" id="disp-actual">0.0</div>
                <div class="sp-lbl">Actual km/h</div>
              </div>
              <div class="sp-box">
                <div class="sp-val" id="disp-target" style="color:var(--accent);">8.0</div>
                <div class="sp-lbl">Target km/h</div>
              </div>
              <div class="sp-box">
                <div class="sp-val" id="disp-ceiling" style="color:var(--success);">20.0</div>
                <div class="sp-lbl">Safety Limit</div>
              </div>
              <div class="sp-box">
                <div class="sp-val" id="disp-cmd">0.0</div>
                <div class="sp-lbl">Final Cmd</div>
              </div>
            </div>

            <div class="preset-btns">
              <button class="p-btn stop" onclick="dispatchSpeed(0.0)">STOP</button>
              <button class="p-btn" onclick="dispatchSpeed(5.0)">5 km/h</button>
              <button class="p-btn" onclick="dispatchSpeed(10.0)">10 km/h</button>
              <button class="p-btn" onclick="dispatchSpeed(15.0)">15 km/h</button>
              <button class="p-btn" onclick="dispatchSpeed(20.0)">20 MAX</button>
            </div>

            <div class="slider-row">
              <button class="step-btn" onclick="adjustSpeedStep(-1.0)">-</button>
              <input type="range" id="speedSlider" min="0" max="20" step="0.5" value="8" oninput="onSliderMove(this.value)" onchange="dispatchSpeed(this.value)">
              <button class="step-btn" onclick="adjustSpeedStep(1.0)">+</button>
              <span id="sliderValLabel" style="font-weight:700; width:55px; text-align:right;">8.0 km/h</span>
            </div>
          </div>
        </div>

        <!-- V2V Coordination & Safety -->
        <div class="panel-section">
          <div class="sec-hdr">
            <span>V2V Fleet Coordination & Safety Matrix</span>
          </div>

          <div class="v2v-box">
            <div class="v2v-header">
              <span style="font-weight:700; font-size:12px;">COORDINATION STATUS</span>
              <span class="v2v-state-tag" id="v2vStateTag">NORMAL</span>
            </div>
            <div class="v2v-desc" id="v2vDescText">Cruising in autonomous lane. Forward haul corridor clear.</div>
          </div>

          <table class="tele-table">
            <tr>
              <td class="t-lbl">Forward Leader</td>
              <td class="t-val" id="t-leader">None</td>
            </tr>
            <tr>
              <td class="t-lbl">Distance to Leader</td>
              <td class="t-val" id="t-leader-dist">--</td>
            </tr>
            <tr>
              <td class="t-lbl">Collision Risk Level</td>
              <td class="t-val"><span class="badge badge-CLEAR" id="t-risk">CLEAR</span></td>
            </tr>
            <tr>
              <td class="t-lbl">LiDAR Obstacle Distance</td>
              <td class="t-val" id="t-lidar-dist">50.0 m</td>
            </tr>
            <tr>
              <td class="t-lbl">Time to Collision (TTC)</td>
              <td class="t-val" id="t-ttc">SAFE</td>
            </tr>
            <tr>
              <td class="t-lbl">Driver Safety Advisory</td>
              <td class="t-val" id="t-advisory" style="font-size:10px; color:#475569;">HAUL ROAD CLEAR</td>
            </tr>
          </table>
        </div>

        <!-- Spatial & Nav Telemetry Table -->
        <div class="panel-section">
          <div class="sec-hdr">
            <span>Kinematics, GPS & Navigation</span>
            <div class="sensor-list" id="sensorList"></div>
          </div>

          <table class="tele-table">
            <tr>
              <td class="t-lbl">Cycle Phase</td>
              <td class="t-val" id="t-cycle-phase" style="color:var(--accent);">HAULING_TO_LOAD</td>
            </tr>
            <tr>
              <td class="t-lbl">Waypoint Target</td>
              <td class="t-val" id="t-wp-target">WP0 (0.0, 87.5, 0.0)</td>
            </tr>
            <tr>
              <td class="t-lbl">Distance to Waypoint</td>
              <td class="t-val" id="t-dist-wp">0.0 m</td>
            </tr>
            <tr>
              <td class="t-lbl">Position (X, Y, Z)</td>
              <td class="t-val" id="t-pos-xyz">-60.0, 87.5, 0.0 m</td>
            </tr>
            <tr>
              <td class="t-lbl">Compass Heading</td>
              <td class="t-val" id="t-heading">0.0°</td>
            </tr>
            <tr>
              <td class="t-lbl">GPS Coordinates</td>
              <td class="t-val" id="t-gps">21.916700° N, 85.333300° E</td>
            </tr>
            <tr>
              <td class="t-lbl">IMU Pitch / Roll</td>
              <td class="t-val" id="t-imu">0.0° / 0.0°</td>
            </tr>
          </table>
        </div>
      </div>
    </div>

    <!-- Bottom Event & Safety Log Table -->
    <div class="event-log-panel">
      <div style="font-size:11px; font-weight:800; color:var(--muted); text-transform:uppercase; margin-bottom:4px;">
        Live Fleet Event & Coordination Log
      </div>
      <table class="log-table">
        <thead>
          <tr>
            <th style="width:70px;">TIME</th>
            <th style="width:90px;">VEHICLE</th>
            <th style="width:130px;">CATEGORY</th>
            <th>EVENT MESSAGE</th>
            <th style="width:80px;">SEVERITY</th>
          </tr>
        </thead>
        <tbody id="logTableBody">
          <!-- Populated by JS -->
        </tbody>
      </table>
    </div>
  </main>

  <script>
    let selectedVehicle = "hemm_01";
    let cameraMode = "rgb"; // "rgb" or "thermal"
    let telemetryData = null;

    // Canvas Map variables
    let mapCanvas = document.getElementById("mineMap");
    let ctx = mapCanvas.getContext("2d");
    let mapScale = 3.2; // pixels per meter
    let mapOffsetX = 0;
    let mapOffsetY = 0;

    // Clock
    setInterval(() => {
      let now = new Date();
      document.getElementById("liveClock").innerText = now.toTimeString().split(" ")[0] + " LOCAL";
    }, 1000);

    function selectVehicle(vid) {
      selectedVehicle = vid;
      document.querySelectorAll(".v-tab").forEach(tab => {
        tab.classList.toggle("active", tab.id === `tab-${vid}`);
      });
      document.getElementById("camOverlayTag").innerText = `${vid.toUpperCase()} ${cameraMode.toUpperCase()}`;
      updateUI();
    }

    function setCameraMode(mode) {
      cameraMode = mode;
      document.getElementById("btn-cam-rgb").classList.toggle("active", mode === "rgb");
      document.getElementById("btn-cam-thermal").classList.toggle("active", mode === "thermal");
      document.getElementById("cam-active-mode-label").innerText = mode === "rgb" ? "OPTICAL RGB" : "THERMAL FLIR IR";
      document.getElementById("camOverlayTag").innerText = `${selectedVehicle.toUpperCase()} ${mode.toUpperCase()}`;
    }

    function onSliderMove(val) {
      document.getElementById("sliderValLabel").innerText = `${parseFloat(val).toFixed(1)} km/h`;
    }

    function adjustSpeedStep(delta) {
      let slider = document.getElementById("speedSlider");
      let newVal = Math.max(0, Math.min(20, parseFloat(slider.value) + delta));
      slider.value = newVal;
      onSliderMove(newVal);
      dispatchSpeed(newVal);
    }

    async function dispatchSpeed(kmh) {
      kmh = parseFloat(kmh);
      document.getElementById("speedSlider").value = kmh;
      onSliderMove(kmh);
      try {
        await fetch("/api/speed", {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({vehicle_id: selectedVehicle, speed_kmh: kmh})
        });
      } catch (e) {
        console.error("Failed to dispatch speed", e);
      }
    }

    function resetMapZoom() {
      mapScale = 3.2;
      mapOffsetX = mapCanvas.width / 2;
      mapOffsetY = mapCanvas.height / 2 - 40;
      drawMineMap();
    }

    function zoomMap(factor) {
      mapScale = Math.max(1.0, Math.min(8.0, mapScale * factor));
      drawMineMap();
    }

    function resizeCanvas() {
      let rect = mapCanvas.parentElement.getBoundingClientRect();
      mapCanvas.width = rect.width;
      mapCanvas.height = rect.height;
      if (mapOffsetX === 0) resetMapZoom();
      drawMineMap();
    }
    window.addEventListener("resize", resizeCanvas);
    setTimeout(resizeCanvas, 100);

    function worldToScreen(x, y) {
      // In Gazebo: X is east(+)/west(-), Y is north(+)/south(-)
      // On Canvas: screenX = offset + X * scale, screenY = offset - Y * scale
      return {
        x: mapOffsetX + x * mapScale,
        y: mapOffsetY - (y - 50) * mapScale
      };
    }

    function drawMineMap() {
      if (!telemetryData || !ctx) return;
      ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);

      let geo = telemetryData.mine;

      // 1. Draw Benches (Concentric Mining Terraces)
      ctx.fillStyle = "#E2E8F0"; // Bench 1 (Top level 0.0m)
      let b1_tl = worldToScreen(-95, 105);
      let b1_br = worldToScreen(95, 60);
      ctx.fillRect(b1_tl.x, b1_tl.y, b1_br.x - b1_tl.x, b1_br.y - b1_tl.y);

      ctx.fillStyle = "#CBD5E1"; // Bench 2 (-7.15m)
      let b2_tl = worldToScreen(-85, 60);
      let b2_br = worldToScreen(85, 45);
      ctx.fillRect(b2_tl.x, b2_tl.y, b2_br.x - b2_tl.x, b2_br.y - b2_tl.y);

      ctx.fillStyle = "#94A3B8"; // Bench 3 Pit Floor (-14.15m)
      let b3_tl = worldToScreen(-50, 45);
      let b3_br = worldToScreen(50, -25);
      ctx.fillRect(b3_tl.x, b3_tl.y, b3_br.x - b3_tl.x, b3_br.y - b3_tl.y);

      // 2. Draw Haul Roads & Ramps
      ctx.fillStyle = "#F8FAFC";
      ctx.strokeStyle = "#94A3B8";
      ctx.lineWidth = 1.5;

      // Bench 1 Haul Road (18m wide)
      let r_apron = worldToScreen(-90, 87.5 + 9);
      let r_apron_b = worldToScreen(90, 87.5 - 9);
      ctx.fillRect(r_apron.x, r_apron.y, r_apron_b.x - r_apron.x, r_apron_b.y - r_apron.y);
      ctx.strokeRect(r_apron.x, r_apron.y, r_apron_b.x - r_apron.x, r_apron_b.y - r_apron.y);

      // Ramp 1 (16m wide, connects Y=87.5 to Y=55)
      let ramp1_tl = worldToScreen(-8, 87.5);
      let ramp1_br = worldToScreen(8, 55);
      ctx.fillStyle = "#F1F5F9";
      ctx.fillRect(ramp1_tl.x, ramp1_tl.y, ramp1_br.x - ramp1_tl.x, ramp1_br.y - ramp1_tl.y);
      ctx.strokeRect(ramp1_tl.x, ramp1_tl.y, ramp1_br.x - ramp1_tl.x, ramp1_br.y - ramp1_tl.y);

      // Ramp 2 (16m wide, connects Y=50 to Y=22)
      let ramp2_tl = worldToScreen(-8, 50);
      let ramp2_br = worldToScreen(8, 22);
      ctx.fillRect(ramp2_tl.x, ramp2_tl.y, ramp2_br.x - ramp2_tl.x, ramp2_br.y - ramp2_tl.y);
      ctx.strokeRect(ramp2_tl.x, ramp2_tl.y, ramp2_br.x - ramp2_tl.x, ramp2_br.y - ramp2_tl.y);

      // Loading Turning Pad (34x34m)
      let lp_tl = worldToScreen(-17, 19);
      let lp_br = worldToScreen(17, -15);
      ctx.fillStyle = "#DBEAFE";
      ctx.fillRect(lp_tl.x, lp_tl.y, lp_br.x - lp_tl.x, lp_br.y - lp_tl.y);
      ctx.strokeStyle = "#3B82F6";
      ctx.strokeRect(lp_tl.x, lp_tl.y, lp_br.x - lp_tl.x, lp_br.y - lp_tl.y);

      // Dumping Zone Pad (26x18m)
      let dp_tl = worldToScreen(17, 87.5 + 9);
      let dp_br = worldToScreen(43, 87.5 - 9);
      ctx.fillStyle = "#FEF3C7";
      ctx.fillRect(dp_tl.x, dp_tl.y, dp_br.x - dp_tl.x, dp_br.y - dp_tl.y);
      ctx.strokeStyle = "#D97706";
      ctx.strokeRect(dp_tl.x, dp_tl.y, dp_br.x - dp_tl.x, dp_br.y - dp_tl.y);

      // Text Labels on Map
      ctx.font = "bold 11px sans-serif";
      ctx.fillStyle = "#1E40AF";
      let lp_text = worldToScreen(0, 2);
      ctx.fillText("LOADING PAD (FLOOR)", lp_text.x - 55, lp_text.y + 4);

      ctx.fillStyle = "#B45309";
      let dp_text = worldToScreen(30, 87.5);
      ctx.fillText("DUMPING PAD", dp_text.x - 35, dp_text.y + 4);

      // Conflict Zones (V2V Deterministic Arbitration Corridors)
      let conflictZones = (geo && geo.conflict_zones) ? geo.conflict_zones : [];
      conflictZones.forEach(cz => {
        let cp = worldToScreen(cz.x, cz.y);
        let cr = cz.radius * mapScale;
        ctx.beginPath();
        ctx.arc(cp.x, cp.y, cr, 0, Math.PI * 2);
        ctx.setLineDash([3, 3]);
        ctx.strokeStyle = "rgba(220, 38, 38, 0.4)";
        ctx.lineWidth = 1.5;
        ctx.stroke();
        ctx.fillStyle = "rgba(239, 68, 68, 0.05)";
        ctx.fill();
        ctx.setLineDash([]);
        ctx.font = "bold 9px monospace";
        ctx.fillStyle = "rgba(185, 28, 28, 0.85)";
        ctx.fillText(cz.code, cp.x - 22, cp.y - cr - 3);
      });

      // 3. Draw V2V Mesh Links between nearby trucks
      let v_list = Object.values(telemetryData.vehicles || {});
      for (let i = 0; i < v_list.length; i++) {
        for (let j = i + 1; j < v_list.length; j++) {
          let vA = v_list[i];
          let vB = v_list[j];
          if (vA.x != null && vA.y != null && vB.x != null && vB.y != null) {
            let d = Math.sqrt((vA.x - vB.x)**2 + (vA.y - vB.y)**2);
            if (d < 35.0) {
              let pA = worldToScreen(vA.x, vA.y);
              let pB = worldToScreen(vB.x, vB.y);
              ctx.beginPath();
              ctx.setLineDash([4, 4]);
              ctx.strokeStyle = "#6366F1";
              ctx.lineWidth = 1.8;
              ctx.moveTo(pA.x, pA.y);
              ctx.lineTo(pB.x, pB.y);
              ctx.stroke();
              ctx.setLineDash([]);

              // Distance badge on link
              let midX = (pA.x + pB.x) / 2;
              let midY = (pA.y + pB.y) / 2;
              ctx.fillStyle = "#4338CA";
              ctx.font = "bold 10px monospace";
              ctx.fillText(`${d.toFixed(1)}m`, midX - 12, midY - 4);
            }
          }
        }
      }

      // 4. Draw Vehicles (HEMM Dumpers)
      v_list.forEach(v => {
        if (v.x == null || v.y == null) return;
        let pt = worldToScreen(v.x, v.y);
        let truckW = 8.5 * mapScale; // length
        let truckH = 4.2 * mapScale; // width
        let yaw = v.yaw || 0.0;
        let speed = (v.speed_kmh != null) ? v.speed_kmh : ((v.speed_mps || 0.0) * 3.6);
        let heading = (v.heading_deg != null) ? v.heading_deg : Math.round(((yaw * 180 / Math.PI) + 360) % 360);
        let risk = (v.safety && v.safety.risk_level) ? v.safety.risk_level : "CLEAR";

        ctx.save();
        ctx.translate(pt.x, pt.y);
        ctx.rotate(-yaw); // Invert because screen Y is flipped

        // Safety buffer radius
        ctx.beginPath();
        ctx.arc(0, 0, 7.0 * mapScale, 0, Math.PI * 2);
        if (risk === "CRITICAL") ctx.strokeStyle = "rgba(239, 68, 68, 0.75)";
        else if (risk === "WARNING") ctx.strokeStyle = "rgba(245, 158, 11, 0.75)";
        else if (risk === "CAUTION") ctx.strokeStyle = "rgba(234, 179, 8, 0.55)";
        else ctx.strokeStyle = "rgba(34, 197, 94, 0.35)";
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Highlight selected vehicle
        if (v.id === selectedVehicle) {
          ctx.strokeStyle = "#0F172A";
          ctx.lineWidth = 2.5;
          ctx.strokeRect(-truckW/2 - 4, -truckH/2 - 4, truckW + 8, truckH + 8);
        }

        // Truck Body (Yellow, Blue, Green based on vehicle colors)
        ctx.fillStyle = v.color || (v.id === "hemm_01" ? "#EAB308" : (v.id === "hemm_02" ? "#2563EB" : "#16A34A"));
        ctx.fillRect(-truckW/2, -truckH/2, truckW, truckH);
        ctx.strokeStyle = "#0F172A";
        ctx.lineWidth = 1.8;
        ctx.strokeRect(-truckW/2, -truckH/2, truckW, truckH);

        // Cab Front indicator & Forward Arrow
        ctx.fillStyle = "#1E293B";
        ctx.fillRect(truckW/6, -truckH/2, truckW/3, truckH);

        // Direction Arrow in vehicle frame pointing forward (+X)
        ctx.fillStyle = "#FFFFFF";
        ctx.beginPath();
        ctx.moveTo(truckW/2 + 2, 0);
        ctx.lineTo(truckW/4, -truckH/3);
        ctx.lineTo(truckW/4, truckH/3);
        ctx.closePath();
        ctx.fill();

        ctx.restore();

        // Top Label: ID, Speed, Heading
        ctx.font = "bold 11px sans-serif";
        let topLabel = `${v.label || v.id.toUpperCase()} • ${speed.toFixed(1)} km/h • ${Math.round(heading)}°`;
        let topMetrics = ctx.measureText(topLabel);
        // Semi-transparent backing pill for top label
        ctx.fillStyle = "rgba(255, 255, 255, 0.88)";
        ctx.fillRect(pt.x - topMetrics.width / 2 - 4, pt.y - truckH - 18, topMetrics.width + 8, 15);
        ctx.fillStyle = "#0F172A";
        ctx.fillText(topLabel, pt.x - topMetrics.width / 2, pt.y - truckH - 6);

        // Bottom Pill: Risk Level badge & V2V state
        let v2vSt = v.v2v_state || "NORMAL";
        let bottomText = `[${risk}] ${v2vSt}`;
        ctx.font = "bold 9px monospace";
        let botMetrics = ctx.measureText(bottomText);

        let pillBg = "#475569";
        if (risk === "CRITICAL" || v2vSt.includes("WAITING")) pillBg = "#DC2626";
        else if (risk === "WARNING") pillBg = "#D97706";
        else if (v2vSt.includes("GO") || v2vSt === "CROSSING") pillBg = "#16A34A";
        else if (v2vSt === "CLEAR") pillBg = "#0284C7";
        else if (v2vSt.includes("RESUMING")) pillBg = "#2563EB";

        ctx.fillStyle = pillBg;
        ctx.fillRect(pt.x - botMetrics.width / 2 - 5, pt.y + truckH + 4, botMetrics.width + 10, 15);
        ctx.fillStyle = "#FFFFFF";
        ctx.fillText(bottomText, pt.x - botMetrics.width / 2, pt.y + truckH + 15);
      });
    }

    function updateUI() {
      if (!telemetryData) return;

      let v_list = Object.values(telemetryData.vehicles);

      // 1. Update Header KPIs
      document.getElementById("kpi-fleet-count").innerText = `${v_list.length} / 3`;
      let meanSpeed = v_list.reduce((acc, v) => acc + v.speed_kmh, 0) / (v_list.length || 1);
      document.getElementById("kpi-mean-speed").innerHTML = `${meanSpeed.toFixed(1)} <span style="font-size:12px;">km/h</span>`;

      // Cycle distribution
      let phases = v_list.map(v => v.cycle_phase);
      let loadCount = phases.filter(p => p === "LOADING").length;
      let dumpCount = phases.filter(p => p === "DUMPING").length;
      let haulCount = phases.filter(p => p.includes("HAUL") || p.includes("RAMP")).length;
      document.getElementById("kpi-cycles").innerText = `H:${haulCount} | L:${loadCount} | D:${dumpCount}`;

      // Overall Safety
      let hasCrit = v_list.some(v => v.safety.risk_level === "CRITICAL");
      let hasWarn = v_list.some(v => v.safety.risk_level === "WARNING");
      let sElem = document.getElementById("kpi-safety-status");
      if (hasCrit) { sElem.innerText = "CRITICAL ALERT"; sElem.style.color = "var(--danger)"; }
      else if (hasWarn) { sElem.innerText = "CAUTION"; sElem.style.color = "var(--warning)"; }
      else { sElem.innerText = "ALL CLEAR"; sElem.style.color = "var(--success)"; }

      let minDist = Math.min(...v_list.map(v => v.safety.obstacle_dist));
      document.getElementById("kpi-min-dist").innerHTML = `${minDist.toFixed(1)} <span style="font-size:12px;">m</span>`;

      // 2. Update Selected Vehicle Panel
      let sel = telemetryData.vehicles[selectedVehicle];
      if (!sel) return;

      // Speed Dashboard
      document.getElementById("disp-actual").innerText = sel.speed_kmh.toFixed(1);
      document.getElementById("disp-target").innerText = sel.target_speed_kmh.toFixed(1);
      let ceilingKmh = (sel.safety.risk_level === "CRITICAL") ? 0.0 : ((sel.safety.risk_level === "WARNING") ? 3.0 : ((sel.safety.risk_level === "CAUTION") ? 8.0 : 20.0));
      document.getElementById("disp-ceiling").innerText = ceilingKmh.toFixed(1);
      let cmdKmh = Math.min(sel.target_speed_kmh, ceilingKmh);
      document.getElementById("disp-cmd").innerText = cmdKmh.toFixed(1);

      document.getElementById("ctrl-reason").innerText = (sel.safety.risk_level === "CRITICAL") ?
        "LIDAR SAFETY CRITICAL STOP" : ((sel.v2v_state === "WAITING") ? "V2V QUEUE WAITING" : "OPERATOR CRUISING SPEED");

      // V2V Box
      let vTag = document.getElementById("v2vStateTag");
      vTag.innerText = sel.v2v_state;
      if (sel.v2v_state.includes("WAITING")) vTag.style.background = "#DC2626";
      else if (sel.v2v_state.includes("GO") || sel.v2v_state === "CROSSING") vTag.style.background = "#16A34A";
      else if (sel.v2v_state === "CLEAR") vTag.style.background = "#0284C7";
      else if (sel.v2v_state.includes("RESUMING")) vTag.style.background = "#2563EB";
      else if (sel.v2v_state === "FOLLOWING") vTag.style.background = "#3B82F6";
      else vTag.style.background = "#64748B";

      let vDesc = `Phase: ${sel.cycle_phase} | Lane: Haul Route Corridor`;
      if (sel.v2v_state.includes("WAITING")) vDesc = `${sel.v2v_state}: Stopped at safe buffer (linear.x=0, angular.z=0).`;
      else if (sel.v2v_state.includes("GO") || sel.v2v_state === "CROSSING") vDesc = `Right-of-way granted for conflict zone corridor.`;
      else if (sel.v2v_state === "CLEAR") vDesc = `Conflict zone cleared. Resuming normal haul cycle.`;
      else if (sel.v2v_state.includes("RESUMING")) vDesc = `Conflict cleared. Released to resume cruising speed.`;
      else if (sel.v2v_state === "FOLLOWING") vDesc = `Maintaining safe headway behind ${sel.leader_id || 'leader'} at ${sel.dist_to_leader_m || 0}m.`;
      document.getElementById("v2vDescText").innerText = vDesc;

      document.getElementById("t-leader").innerText = sel.leader_id || "None";
      document.getElementById("t-leader-dist").innerText = sel.dist_to_leader_m ? `${sel.dist_to_leader_m.toFixed(1)} m` : "--";

      let rBadge = document.getElementById("t-risk");
      rBadge.innerText = sel.safety.risk_level;
      rBadge.className = `badge badge-${sel.safety.risk_level}`;

      document.getElementById("t-lidar-dist").innerText = `${sel.safety.obstacle_dist.toFixed(1)} m`;
      document.getElementById("t-ttc").innerText = sel.safety.ttc_s ? `${sel.safety.ttc_s.toFixed(1)} s` : "SAFE";
      document.getElementById("t-advisory").innerText = sel.safety.advisory;

      // Kinematics & Nav Table
      document.getElementById("t-cycle-phase").innerText = sel.cycle_phase;
      document.getElementById("t-wp-target").innerText = `WP${sel.wp_index} (${sel.next_wp[0].toFixed(1)}, ${sel.next_wp[1].toFixed(1)})`;
      document.getElementById("t-dist-wp").innerText = `${sel.dist_to_wp.toFixed(1)} m`;
      document.getElementById("t-pos-xyz").innerText = `${sel.x.toFixed(1)}, ${sel.y.toFixed(1)}, ${sel.z.toFixed(1)} m`;
      document.getElementById("t-heading").innerText = `${sel.heading_deg}°`;
      document.getElementById("t-gps").innerText = `${sel.gps.lat.toFixed(6)}° N, ${sel.gps.lon.toFixed(6)}° E`;
      document.getElementById("t-imu").innerText = `${sel.imu.pitch}° / ${sel.imu.roll}°`;

      // Sensor Status tags
      let sHtml = '';
      ['gps', 'lidar', 'imu', 'camera', 'thermal', 'odom'].forEach(s => {
        let on = sel.sensors[s];
        sHtml += `<span class="sens-tag ${on ? 'sens-on' : 'sens-off'}">${s.toUpperCase()}</span>`;
      });
      document.getElementById("sensorList").innerHTML = sHtml;

      // 3. Update Event Log
      if (telemetryData.events) {
        let logHtml = '';
        telemetryData.events.slice(0, 8).forEach(ev => {
          logHtml += `<tr>
            <td>${ev.time}</td>
            <td><b>${ev.vehicle}</b></td>
            <td>${ev.category}</td>
            <td>${ev.message}</td>
            <td class="sev-${ev.severity}">${ev.severity}</td>
          </tr>`;
        });
        document.getElementById("logTableBody").innerHTML = logHtml;
      }

      drawMineMap();
    }

    // Telemetry Polling (4 Hz)
    async function fetchTelemetry() {
      try {
        let res = await fetch('/api/telemetry');
        if (res.ok) {
          telemetryData = await res.json();
          updateUI();
        }
      } catch (e) {}
    }
    setInterval(fetchTelemetry, 250);

    // Camera Stream Refresh (3 FPS)
    setInterval(() => {
      let img = document.getElementById("cameraStream");
      img.src = `/api/camera/${selectedVehicle}?type=${cameraMode}&t=${Date.now()}`;
    }, 333);
  </script>
</body>
</html>
"""


class DashboardHTTPHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_HEAD(self):
        self.send_response(200)
        self.send_header("Content-type", "text/html; charset=utf-8")
        self.end_headers()

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)

        if path in ["/", "/index.html"]:
            self.send_response(200)
            self.send_header("Content-type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(DASHBOARD_HTML.encode("utf-8"))

        elif path == "/api/telemetry":
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            payload = {
                "vehicles": ros_node.state,
                "mine": MINE_GEOMETRY,
                "events": list(ros_node.events),
                "server_time": time.time()
            }
            self.wfile.write(json.dumps(payload).encode("utf-8"))

        elif path.startswith("/api/camera/"):
            vid = path.split("/api/camera/")[1]
            cam_type = query.get("type", ["rgb"])[0]

            if cam_type == "thermal":
                img_bytes = ros_node.latest_thermal_images.get(vid)
            else:
                img_bytes = ros_node.latest_rgb_images.get(vid)

            if img_bytes:
                self.send_response(200)
                self.send_header("Content-type", "image/jpeg")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(img_bytes)
            else:
                self.send_response(204)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/speed":
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length)
            try:
                data = json.loads(body.decode('utf-8'))
                vid = data.get("vehicle_id")
                speed_kmh = float(data.get("speed_kmh", 8.0))
                ros_node.set_vehicle_speed(vid, speed_kmh)
                self.send_response(200)
                self.send_header("Content-type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "ok", "vehicle_id": vid, "speed_kmh": speed_kmh}).encode("utf-8"))
            except Exception:
                self.send_response(400)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()


def start_http_server(port=8080):
    server = HTTPServer(('0.0.0.0', port), DashboardHTTPHandler)
    print(f"\n=======================================================")
    print(f"  MINEGUARD AI CONTROL ROOM DASHBOARD RUNNING")
    print(f"  URL: http://localhost:{port} or http://127.0.0.1:{port}")
    print(f"=======================================================\n", flush=True)
    server.serve_forever()


def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = DashboardROSNode()

    http_thread = threading.Thread(target=start_http_server, args=(8080,), daemon=True)
    http_thread.start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
