#!/usr/bin/env python3
"""
SIH Mine Safety — Namespaced Driver Assistance & Collision Warning System
========================================================================
Supports multiple HEMM instances via --vehicle-id argument.
Each instance subscribes/publishes to its own namespaced topics.

Usage:
  python3 hemm_safety_system.py --vehicle-id hemm_01
  python3 hemm_safety_system.py --vehicle-id hemm_02
  python3 hemm_safety_system.py --vehicle-id hemm_03

Topics subscribed (example for hemm_01):
  /hemm_01/scan, /hemm_01/odom, /hemm_01/imu, /hemm_01/gps

Topics published (example for hemm_01):
  /hemm_01/safety/risk_level
  /hemm_01/safety/warning
  /hemm_01/safety/obstacle_distance
========================================================================
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
import math
import json
import time
import argparse
import sys
SPAWN_POSES = {
    "hemm_01": (-60.0, 85.0, 0.0, 0.0),
    "hemm_02": ( 55.0, 91.5, 0.0, math.pi),
    "hemm_03": (  5.0, 91.5, 0.0, math.pi),
}


class HEMMSafetySystem(Node):
    def __init__(self, vehicle_id: str):
        # Sanitize vehicle_id for ROS 2 node name (replace - with _)
        node_name = f"safety_{vehicle_id.replace('-', '_')}"
        super().__init__(node_name)

        self.vehicle_id = vehicle_id
        ns = f"/{vehicle_id}"

        self.get_logger().info(f"Starting Safety System for {vehicle_id} | Namespace: {ns}")

        # Subscriptions
        self.create_subscription(LaserScan,  f"{ns}/scan", self.scan_callback, 10)
        self.create_subscription(Odometry,   f"{ns}/odom", self.odom_callback, 10)
        self.create_subscription(Imu,        f"{ns}/imu",  self.imu_callback,  10)
        self.create_subscription(NavSatFix,  f"{ns}/gps",  self.gps_callback,  10)

        # Publishers
        self.pub_warning_json  = self.create_publisher(String,  f"{ns}/safety/warning",           10)
        self.pub_obstacle_dist = self.create_publisher(Float32, f"{ns}/safety/obstacle_distance",  10)
        self.pub_risk_level    = self.create_publisher(String,  f"{ns}/safety/risk_level",         10)

        # Vehicle state
        self.vx          = 0.0
        self.speed_mps   = 0.0
        self.speed_kmh   = 0.0
        self.is_reversing= False
        self.gps_lat     = 21.9167
        self.gps_lon     = 85.3333
        self.x           = 0.0
        self.y           = 0.0
        self.pitch_deg   = 0.0
        self.roll_deg    = 0.0

        # Sector distances
        self.sectors = {
            'front': 50.0, 'front_corridor': 50.0,
            'front_left': 50.0, 'front_right': 50.0,
            'left': 50.0, 'right': 50.0, 'rear': 50.0
        }
        self.active_hazard_dist   = 50.0
        self.closest_dist         = 50.0
        self.closest_bearing_deg  = 0.0
        self.closest_sector       = 'front'

        # Risk state
        self.risk_level    = "CLEAR"
        self.risk_code     = 0
        self.driver_advisory = "HAUL ROAD CLEAR — MAINTAIN SAFE OPERATION"
        self.ttc_seconds   = None
        self.gps_received  = False

        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info(f"{vehicle_id.upper()} Safety System INITIALIZED")

    # ------------------------------------------------------------------ callbacks

    def odom_callback(self, msg: Odometry):
        self.vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed_mps = math.sqrt(self.vx**2 + vy**2)
        self.speed_kmh = self.speed_mps * 3.6
        self.is_reversing = (self.vx < -0.2)
        if not self.gps_received:
            ox = msg.pose.pose.position.x
            oy = msg.pose.pose.position.y
            sx, sy, sz, syaw = SPAWN_POSES.get(self.vehicle_id, (0.0, 0.0, 0.0, 0.0))
            cos_s = math.cos(syaw)
            sin_s = math.sin(syaw)
            self.x = sx + (ox * cos_s - oy * sin_s)
            self.y = sy + (ox * sin_s + oy * cos_s)

    def imu_callback(self, msg: Imu):
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        sinp = 2 * (qw * qy - qz * qx)
        self.pitch_deg = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
        sinr = 2 * (qw * qx + qy * qz)
        cosr = 1 - 2 * (qx**2 + qy**2)
        self.roll_deg = math.degrees(math.atan2(sinr, cosr))

    def gps_callback(self, msg: NavSatFix):
        self.gps_lat = msg.latitude
        self.gps_lon = msg.longitude
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.x = (msg.longitude - 85.3333) * 103275.0
            self.y = (msg.latitude - 21.9167) * 111320.0
            self.gps_received = True

    def scan_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        s_front, s_corridor, s_front_left, s_front_right = [], [], [], []
        s_left, s_right, s_rear = [], [], []
        overall_min = 50.0
        overall_bearing = 0.0
        CORRIDOR_HALF_WIDTH = 1.9

        # Ramp geometry parameters for ramp-to-flat and flat-to-ramp transitions:
        # Ramps 1 and 2 run between y in [14.0, 85.0] with slope ~14.5 deg (0.25 rad).
        # Transitions extend into pit floor (y >= 5.0) and Bench 1 (y <= 88.0).
        in_ramp_corridor = (abs(self.x) <= 10.0 and 0.0 <= self.y <= 88.0)
        alpha = abs(self.pitch_deg)
        effective_pitch = max(alpha, 14.5) if in_ramp_corridor else alpha

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            deg = math.degrees(angle)
            px = r * math.cos(angle)
            py = r * math.sin(angle)

            # 1. Self-mask: truck body behind sensor and front wheel arches / cabin frame.
            # Sensor is mounted at x=4.05 on front bumper.
            # Front wheels and bumper arch extend backward from x=1.0 to -8.5m with half-width 2.35m.
            if px <= 1.0 and abs(py) <= 2.35 and px >= -8.5:
                continue
            if r < 1.1:
                continue
            # Mask steered front tire / bumper arch glances
            if px <= 1.85 and abs(py) <= 1.2 and abs(deg) > 20.0:
                continue
            # Mask near side tire reflections at sharp angles
            if r < 2.5 and abs(deg) > 50.0 and px < 1.5:
                continue

            # 2. Rigorous analytical ramp & transition ground strike compensation:
            # Planar road pavement returns occur at r in [1.2, 4.8m] when pitching or on ramps.
            if (effective_pitch > 1.2 or in_ramp_corridor) and abs(deg) < 65.0:
                cos_az = math.cos(angle)
                if cos_az > 0.10:
                    pitch_angle = max(effective_pitch, 14.0) if in_ramp_corridor else effective_pitch
                    r_ground = 0.85 / (math.sin(math.radians(max(pitch_angle, 1.2))) * cos_az)
                    # Filter returns that match the planar road pavement surface.
                    # Genuine physical obstacles (< 1.2m or distinctly different distance) are preserved.
                    if (r_ground - 2.0) <= r <= (r_ground + 1.50) and r >= 1.2:
                        continue

            if r < overall_min:
                overall_min = r
                overall_bearing = deg

            # 3. Forward driving corridor: strictly ahead of bumper within lane width
            if px > 1.8 and abs(py) <= 1.5 and abs(deg) <= 26.0:
                s_corridor.append(r)

            if   -30.0 <= deg <=  30.0: s_front.append(r)
            elif  30.0 <  deg <=  70.0: s_front_left.append(r)
            elif -70.0 <= deg <  -30.0: s_front_right.append(r)
            elif  70.0 <  deg <= 120.0: s_left.append(r)
            elif -120.0 <= deg <  -70.0: s_right.append(r)
            else:                         s_rear.append(r)

        self.sectors['front']          = min(s_front)       if s_front       else 50.0
        self.sectors['front_corridor'] = min(s_corridor)    if s_corridor    else 50.0
        self.sectors['front_left']     = min(s_front_left)  if s_front_left  else 50.0
        self.sectors['front_right']    = min(s_front_right) if s_front_right else 50.0
        self.sectors['left']           = min(s_left)        if s_left        else 50.0
        self.sectors['right']          = min(s_right)       if s_right       else 50.0
        self.sectors['rear']           = min(s_rear)        if s_rear        else 50.0

        self.closest_dist = overall_min
        self.closest_bearing_deg = overall_bearing

        min_sec_dist = 50.0
        for s_name, s_dist in self.sectors.items():
            if s_name == 'front_corridor':
                continue
            if s_dist < min_sec_dist:
                min_sec_dist = s_dist
                self.closest_sector = s_name

        # Forward driving hazard is governed by the driving corridor
        forward_path_dist = self.sectors['front_corridor']

        if self.is_reversing:
            self.active_hazard_dist = self.sectors['rear']
            closing_speed = abs(self.vx)
            self.ttc_seconds = (self.active_hazard_dist / closing_speed) if closing_speed > 0.3 else None
        else:
            # Forward collision hazard is primarily what lies within the vehicle forward corridor
            forward_hazard = forward_path_dist
            # Flank side-swipe: only trigger hazard if dangerously close (< 0.65m)
            flank_min = min(self.sectors['front_left'], self.sectors['front_right'])
            if flank_min < 0.65:
                self.active_hazard_dist = min(forward_hazard, flank_min)
            else:
                self.active_hazard_dist = forward_hazard

            closing_speed = self.vx if self.vx > 0.2 else 0.0
            self.ttc_seconds = (forward_path_dist / closing_speed) if closing_speed > 0.3 else None

        d = self.active_hazard_dist
        ttc = self.ttc_seconds

        if d <= 3.5 or (ttc is not None and ttc <= 1.8):
            self.risk_level = "CRITICAL"; self.risk_code = 4
            self.driver_advisory = ("CRITICAL: REAR OBSTACLE IMMINENT! STOP IMMEDIATELY!"
                                    if self.is_reversing else
                                    "CRITICAL: OBSTACLE IMMINENT! APPLY BRAKES IMMEDIATELY!")
        elif d <= 7.0 or (ttc is not None and ttc <= 3.5):
            self.risk_level = "WARNING";  self.risk_code = 3
            self.driver_advisory = ("WARNING: REAR HAZARD CLOSE! REDUCE SPEED!"
                                    if self.is_reversing else
                                    "WARNING: HAZARD CLOSE AHEAD! REDUCE SPEED!")
        elif d <= 14.0 or (ttc is not None and ttc <= 6.0):
            self.risk_level = "CAUTION";  self.risk_code = 2
            self.driver_advisory = "CAUTION: OBSTACLE DETECTED IN HAUL PATH"
        elif d <= 25.0:
            self.risk_level = "ADVISORY"; self.risk_code = 1
            self.driver_advisory = "ADVISORY: OBJECT IN VICINITY — MONITOR PATH"
        else:
            self.risk_level = "CLEAR";    self.risk_code = 0
            self.driver_advisory = "HAUL ROAD CLEAR — MAINTAIN SAFE OPERATION"

        self._publish()

    # ------------------------------------------------------------------ publishing

    def _publish(self):
        d_msg = Float32(); d_msg.data = float(self.active_hazard_dist)
        self.pub_obstacle_dist.publish(d_msg)

        r_msg = String(); r_msg.data = self.risk_level
        self.pub_risk_level.publish(r_msg)

        payload = {
            "vehicle_id":          self.vehicle_id,
            "timestamp":           time.time(),
            "risk_level":          self.risk_level,
            "risk_code":           self.risk_code,
            "driver_advisory":     self.driver_advisory,
            "obstacle_distance_m": round(self.active_hazard_dist, 2),
            "front_obstacle_m":    round(self.sectors['front'], 2),
            "corridor_obstacle_m": round(self.sectors['front_corridor'], 2),
            "front_left_m":        round(self.sectors['front_left'], 2),
            "front_right_m":       round(self.sectors['front_right'], 2),
            "left_m":              round(self.sectors['left'], 2),
            "right_m":             round(self.sectors['right'], 2),
            "rear_m":              round(self.sectors['rear'], 2),
            "closest_distance_m":  round(self.closest_dist, 2),
            "closest_sector":      self.closest_sector,
            "vehicle_speed_kmh":   round(self.speed_kmh, 1),
            "is_reversing":        self.is_reversing,
            "ttc_seconds":         round(self.ttc_seconds, 1) if self.ttc_seconds is not None else None,
            "gps_lat":             round(self.gps_lat, 6),
            "gps_lon":             round(self.gps_lon, 6),
            "road_pitch_deg":      round(self.pitch_deg, 1),
        }
        w_msg = String(); w_msg.data = json.dumps(payload)
        self.pub_warning_json.publish(w_msg)

    def timer_callback(self):
        ttc_str = f"{self.ttc_seconds:.1f}s" if self.ttc_seconds is not None else "N/A"
        colors = {0: "\033[32m", 1: "\033[34m", 2: "\033[33m", 3: "\033[35m", 4: "\033[31;1m"}
        color = colors.get(self.risk_code, "\033[32m")
        reset = "\033[0m"

        vid = self.vehicle_id.upper()
        print(f"\n{'='*60}")
        print(f"  {vid} SAFETY SYSTEM")
        print(f"{'='*60}")
        print(f"  Speed: {self.speed_kmh:4.1f} km/h {'(REV)' if self.is_reversing else '      '}"
              f" | Pitch: {self.pitch_deg:+.1f}°")
        print(f"  Front Lane: {self.sectors['front_corridor']:5.1f}m"
              f" | Front:      {self.sectors['front']:5.1f}m")
        print(f"  FL: {self.sectors['front_left']:5.1f}m"
              f" | FR: {self.sectors['front_right']:5.1f}m"
              f" | Rear: {self.sectors['rear']:5.1f}m")
        print(f"  HAZARD: {self.active_hazard_dist:5.1f}m | TTC: {ttc_str}")
        print(f"  {color}[ {self.risk_level} ]{reset} — {color}{self.driver_advisory}{reset}")
        print(f"{'='*60}", flush=True)


def main(argv=None):
    # Parse --vehicle-id before rclpy.init consumes argv
    parser = argparse.ArgumentParser(description="HEMM Safety System")
    parser.add_argument("--vehicle-id", required=True,
                        choices=["hemm_01", "hemm_02", "hemm_03"],
                        help="Vehicle namespace identifier")
    # rclpy may pass extra ROS args — ignore unknown
    args, ros_args = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    rclpy.init(args=ros_args)
    node = HEMMSafetySystem(args.vehicle_id)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
