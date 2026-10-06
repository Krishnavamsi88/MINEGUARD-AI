#!/usr/bin/env python3
"""
SIH Mine Safety — Autonomous HEMM Waypoint & V2V Fleet Coordination Controller
==============================================================================
Lightweight, deterministic autonomous driving and V2V coordination for 3 HEMM dumpers.
Uses odometry for waypoint tracking and peer V2V broadcasts on /fleet/vehicle_states.

KEY PRINCIPLES:
- NO heavy external planning stacks; deterministic, fast, industrial-grade control.
- Continuous multi-bench mining cycle: Loading Pad <-> Haul Ramps <-> Dumping Pad.
- V2V Fleet Coordination:
    * NORMAL: Cruising along predefined haul route corridor.
    * FOLLOWING: Headway tracking behind forward vehicle (safe buffer 12-25m).
    * WAITING: Queued at safe distance (8-10m) behind a stopped truck or loading pad.
    * OVERTAKING: Smooth lateral corridor shift (4-5m) on wide 16-18m haul roads past stopped truck.
    * YIELDING: Giving right-of-way at narrow sections or ramps.
    * EMERGENCY_WAIT: Proximity alert when peer or environment triggers CRITICAL.
    * RESUMING: Smoothly re-merging onto center corridor and resuming speed.
- Speed governed hierarchically:
    OPERATOR TARGET SPEED -> ROUTE CURVATURE -> V2V COORDINATION -> SAFETY CEILING -> /cmd_vel
    Safety limit (LiDAR) is strictly non-overridable.

Usage:
  python3 hemm_autonomous_controller.py --vehicle-id hemm_01
  python3 hemm_autonomous_controller.py --vehicle-id hemm_02
  python3 hemm_autonomous_controller.py --vehicle-id hemm_03
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import math
import argparse
import sys
import time
import json


# ============================================================
# SPAWN POSITIONS IN WORLD FRAME (from sih_mine_world_3hemm.sdf)
# ============================================================
SPAWN_POSES = {
    "hemm_01": (-60.0, 85.0, 0.0, 0.0),             # (x, y, z, yaw)
    "hemm_02": ( 55.0, 91.5, 0.0, math.pi),
    "hemm_03": (  5.0, 91.5, 0.0, math.pi),
}

# ============================================================
# DETERMINISTIC CONFLICT ZONES FOR V2V ARBITRATION
# ============================================================
CONFLICT_ZONES = {
    "ZONE_RAMP1_JUNCTION": {
        "name": "Ramp 1 Junction",
        "center": (0.0, 83.0),
        "radius": 9.0,
        "approach_dist": 15.0,
    },
    "ZONE_LOADING": {
        "name": "Pit Loading Zone",
        "center": (0.0, 2.0),
        "radius": 8.0,
        "approach_dist": 14.0,
    },
    "ZONE_DUMPING": {
        "name": "Dumping Zone",
        "center": (30.0, 84.0),
        "radius": 8.0,
        "approach_dist": 14.0,
    },
    "ZONE_RAMP2_JUNCTION": {
        "name": "Ramp 2 Junction",
        "center": (0.0, 51.0),
        "radius": 6.0,
        "approach_dist": 12.0,
    },
}

# ============================================================
# MINE WAYPOINT ROUTES: CONTINUOUS DUAL-LANE FULL LOOP
# Eastbound Lane (South side, Bench 1): Y = 84.0
# Westbound Lane (North side, Bench 1): Y = 91.0
# Separation: 7.0m between lane centerlines (2.8m clear air between passing trucks)
# Ramps: West lane (Descent) X = -2.8, East lane (Ascent) X = +2.8
# ============================================================
WAYPOINT_ROUTES = {
    # HEMM_01: Spawn West -> Bench 1 Eastbound (y=84.0) -> Dumping Zone (Dwell) -> Turnaround Loop
    #          -> Ramp 1 (West Lane) -> Ramp 2 (West Lane) -> Loading Zone (Dwell) -> Ramp 2 (East Lane)
    #          -> Ramp 1 (East Lane) -> Dumping Zone (Dwell) -> Loop
    "hemm_01": [
        (-35.0, 84.0,  0.0),    # WP0: Spawn East transit
        (-10.0, 84.0,  0.0),    # WP1: Bench 1 Eastbound transit
        (  0.0, 84.0,  0.0),    # WP2: Bench 1 Ramp 1 crossing (East Lane)
        ( 10.0, 84.0,  0.0),    # WP3: Haul road east toward dumping
        ( 30.0, 84.0,  0.0),    # WP4: Dumping Zone Pad (DUMPING DWELL)
        ( 33.0, 85.5,  0.0),    # WP5: Turnaround arc entry
        ( 35.0, 88.0,  0.0),    # WP6: Turnaround apex
        ( 33.0, 91.0,  0.0),    # WP7: Turnaround exit into Westbound Lane
        ( 15.0, 91.0,  0.0),    # WP8: Return haul road west (Westbound Lane)
        (  5.0, 91.0,  0.0),    # WP9: Return haul road west connector
        ( -2.8, 91.0,  0.0),    # WP10: Ramp 1 entrance (West Lane)
        ( -2.8, 80.0,  0.0),    # WP11: Ramp 1 crest
        ( -2.8, 74.0, -3.5),    # WP12: Ramp 1 descent mid
        ( -2.8, 55.0, -7.15),   # WP13: Bench 2 top
        ( -2.8, 50.0, -7.15),   # WP14: Bench 2 connector
        ( -2.8, 35.0, -10.5),   # WP15: Ramp 2 descent mid
        ( -2.8, 20.0, -14.15),  # WP16: Pit floor entrance
        ( -1.5,  8.0, -14.15),  # WP17: Loading zone approach
        (  0.0,  2.0, -14.15),  # WP18: Loading zone center (LOADING DWELL)
        (  1.5,  8.0, -14.15),  # WP19: Ascending departure
        (  2.8, 20.0, -14.15),  # WP20: Ramp 2 base (East Lane)
        (  2.8, 35.0, -10.5),   # WP21: Ramp 2 ascent mid
        (  2.8, 50.0, -7.15),   # WP22: Bench 2 connector
        (  2.8, 55.0, -7.15),   # WP23: Ramp 1 base
        (  2.8, 74.0, -3.5),    # WP24: Ramp 1 ascent mid
        (  2.8, 80.0,  0.0),    # WP25: Ramp 1 crest
        (  2.8, 84.0,  0.0),    # WP26: Bench 1 top East lane
        ( 15.0, 84.0,  0.0),    # WP27: Haul road east toward dumping
    ],

    # HEMM_02: Spawn East -> Dumping Zone (Dwell) -> Turnaround Loop -> Ramp 1 (West Lane)
    #          -> Ramp 2 (West Lane) -> Loading Zone (Dwell) -> Ramp 2 (East Lane) -> Ramp 1 -> Dumping -> Loop
    "hemm_02": [
        ( 30.0, 84.0,  0.0),    # WP0: Dumping Zone Pad (DUMPING DWELL)
        ( 33.0, 85.5,  0.0),    # WP1: Turnaround arc entry
        ( 35.0, 88.0,  0.0),    # WP2: Turnaround apex
        ( 33.0, 91.0,  0.0),    # WP3: Turnaround exit into Westbound Lane
        ( 15.0, 91.0,  0.0),    # WP4: Return haul road west (Westbound Lane)
        (  5.0, 91.0,  0.0),    # WP5: Bench 1 loop connector
        ( -2.8, 91.0,  0.0),    # WP6: Ramp 1 entrance (West Lane)
        ( -2.8, 80.0,  0.0),    # WP7: Ramp 1 crest
        ( -2.8, 74.0, -3.5),    # WP8: Ramp 1 descent mid
        ( -2.8, 55.0, -7.15),   # WP9: Bench 2 top
        ( -2.8, 50.0, -7.15),   # WP10: Bench 2 connector
        ( -2.8, 35.0, -10.5),   # WP11: Ramp 2 descent mid
        ( -2.8, 20.0, -14.15),  # WP12: Pit floor entrance
        ( -1.5,  8.0, -14.15),  # WP13: Loading approach
        (  0.0,  2.0, -14.15),  # WP14: Loading zone center (LOADING DWELL)
        (  1.5,  8.0, -14.15),  # WP15: Ascending departure
        (  2.8, 20.0, -14.15),  # WP16: Ramp 2 base (East Lane)
        (  2.8, 35.0, -10.5),   # WP17: Ramp 2 ascent mid
        (  2.8, 50.0, -7.15),   # WP18: Bench 2 connector
        (  2.8, 55.0, -7.15),   # WP19: Ramp 1 base
        (  2.8, 74.0, -3.5),    # WP20: Ramp 1 ascent mid
        (  2.8, 80.0,  0.0),    # WP21: Ramp 1 crest
        (  2.8, 84.0,  0.0),    # WP22: Bench 1 top East lane
        ( 15.0, 84.0,  0.0),    # WP23: Haul road east toward dumping
    ],

    # HEMM_03: Spawn Mid-East -> Ramp 1 (West Lane) -> Ramp 2 (West Lane) -> Loading Zone (Dwell)
    #          -> Ramp 2 (East Lane) -> Ramp 1 (East Lane) -> Dumping Zone (Dwell) -> Turnaround Loop
    "hemm_03": [
        ( -2.8, 91.0,  0.0),    # WP0: Ramp 1 entrance (West Lane)
        ( -2.8, 80.0,  0.0),    # WP1: Ramp 1 crest
        ( -2.8, 74.0, -3.5),    # WP2: Ramp 1 descent mid
        ( -2.8, 55.0, -7.15),   # WP3: Bench 2 top
        ( -2.8, 50.0, -7.15),   # WP4: Bench 2 connector
        ( -2.8, 35.0, -10.5),   # WP5: Ramp 2 descent mid
        ( -2.8, 20.0, -14.15),  # WP6: Pit floor entrance
        ( -1.5,  8.0, -14.15),  # WP7: Loading approach
        (  0.0,  2.0, -14.15),  # WP8: Loading zone center (LOADING DWELL)
        (  1.5,  8.0, -14.15),  # WP9: Ascending departure
        (  2.8, 20.0, -14.15),  # WP10: Ramp 2 base (East Lane)
        (  2.8, 35.0, -10.5),   # WP11: Ramp 2 ascent
        (  2.8, 50.0, -7.15),   # WP12: Bench 2 connector
        (  2.8, 55.0, -7.15),   # WP13: Ramp 1 base
        (  2.8, 74.0, -3.5),    # WP14: Ramp 1 ascent
        (  2.8, 80.0,  0.0),    # WP15: Ramp 1 crest
        (  2.8, 84.0,  0.0),    # WP16: Bench 1 top East lane
        ( 15.0, 84.0,  0.0),    # WP17: Bench 1 east toward dumping
        ( 30.0, 84.0,  0.0),    # WP18: Dumping zone pad (DUMPING DWELL)
        ( 33.0, 85.5,  0.0),    # WP19: Turnaround arc entry
        ( 35.0, 88.0,  0.0),    # WP20: Turnaround apex
        ( 33.0, 91.0,  0.0),    # WP21: Turnaround exit into Westbound Lane
        ( 15.0, 91.0,  0.0),    # WP22: Bench 1 mid return
        (  5.0, 91.0,  0.0),    # WP23: Bench 1 mid loop connector
    ],
}

# Speed ceilings per risk level in m/s (20 km/h = 5.56 m/s)
SPEED_BY_RISK = {
    "CLEAR":    5.56,   # Full operator ceiling up to 20.0 km/h
    "ADVISORY": 3.89,   # Up to 14.0 km/h
    "CAUTION":  2.22,   # Up to 8.0 km/h
    "WARNING":  0.83,   # Crawl speed (3.0 km/h)
    "CRITICAL": 0.0,    # Immediate full stop
}

# ---- Controller parameters ----
WAYPOINT_RADIUS_M       = 3.2    # Radius to consider a waypoint reached
MAX_ANGULAR_VEL         = 0.8    # rad/s max turn rate
HEADING_KP              = 1.2    # Proportional gain for heading error
DWELL_TIME_S            = 5.0    # Seconds to pause at Loading / Dumping pads
CONTROL_HZ              = 10.0   # Control loop frequency
V2V_BROADCAST_HZ        = 5.0    # Fleet coordination broadcast rate

# V2V Coordination Thresholds
V2V_DETECTION_DIST_M    = 28.0   # Lookahead range to detect peer HEMM
V2V_FOLLOW_SAFE_DIST_M  = 16.0   # Safe following headway distance
V2V_STOP_BUFFER_M       = 9.0    # Minimum buffer distance behind leader
OVERTAKE_LATERAL_OFFSET = 4.5    # Lateral shift (m) on 16-18m haul roads

# ============================================================
# DETERMINISTIC CONFLICT ZONES FOR V2V ARBITRATION
# ============================================================
CONFLICT_ZONES = {
    "ZONE_RAMP1_JUNCTION": {
        "name": "Ramp 1 Junction",
        "center": (0.0, 84.0),
        "radius": 11.0,
        "approach_dist": 18.0,
    },
    "ZONE_LOADING": {
        "name": "Pit Loading Zone",
        "center": (0.0, 3.0),
        "radius": 9.0,
        "approach_dist": 16.0,
    },
    "ZONE_DUMPING": {
        "name": "Dumping Zone",
        "center": (30.0, 88.0),
        "radius": 9.0,
        "approach_dist": 16.0,
    },
    "ZONE_RAMP2_JUNCTION": {
        "name": "Ramp 2 Junction",
        "center": (0.0, 51.0),
        "radius": 7.0,
        "approach_dist": 14.0,
    },
}



class HEMMAutonomousController(Node):
    def __init__(self, vehicle_id: str, target_speed: float = 2.2):
        node_name = f"autonomous_{vehicle_id.replace('-', '_')}"
        super().__init__(node_name)

        self.vehicle_id = vehicle_id
        self.ns = f"/{vehicle_id}"

        # Route state
        self.waypoints = list(WAYPOINT_ROUTES[vehicle_id])
        self.current_route = self.waypoints
        self.wp_index = 0
        self.dwell_until = None
        self.cycle_phase = "HAULING" if vehicle_id != "hemm_02" else "DUMPING"

        # Speed settings (persisted from operator)
        self.target_speed = float(target_speed)

        # Vehicle pose (from odometry in world frame)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.current_speed = 0.0
        self.odom_received = False

        # Safety state from LiDAR safety node
        self.risk_level = "CLEAR"

        # V2V State Management & Conflict Zone Arbitration
        self.v2v_state = "NORMAL"
        self.leader_id = None
        self.dist_to_leader = None
        self.lateral_offset = 0.0
        self.overtaking_stage = 0
        self.peer_states = {}      # Map of peer_id -> state dict

        # Conflict Zone Tracking
        self.active_conflict_zone = None
        self.conflict_claim_time = None
        self.conflict_inside = False
        self.current_zone_dist = None
        self.clear_until = 0.0
        self.resuming_until = 0.0
        self.aligning_in_place = False
        self.last_v2v_log_state = None

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(Twist, f"{self.ns}/cmd_vel", 10)
        self.pub_target_speed = self.create_publisher(Float32, f"{self.ns}/target_speed", 10)
        self.pub_nav_status = self.create_publisher(String, f"{self.ns}/nav_status", 10)
        self.pub_vehicle_v2v_state = self.create_publisher(String, f"{self.ns}/v2v/state", 10)

        # V2V Fleet Shared Topic
        self.pub_fleet_v2v = self.create_publisher(String, "/fleet/vehicle_states", 10)
        self.create_subscription(String, "/fleet/vehicle_states", self.fleet_v2v_callback, 10)

        # Vehicle Sensors & Safety Subscriptions
        self.gps_received = False
        self.create_subscription(NavSatFix, f"{self.ns}/gps", self.gps_callback, 10)
        self.create_subscription(Odometry, f"{self.ns}/odom", self.odom_callback, 10)
        self.create_subscription(String, f"{self.ns}/safety/risk_level", self.risk_callback, 10)
        self.create_subscription(Float32, f"{self.ns}/set_target_speed", self.set_target_speed_callback, 10)

        # Control Timers
        self.create_timer(1.0 / CONTROL_HZ, self.control_loop)
        self.create_timer(1.0 / V2V_BROADCAST_HZ, self.broadcast_v2v_state)

        # Navigation telemetry helper
        self.current_wp_target = self.current_route[0]
        self.dist_to_wp = 0.0

        self.get_logger().info(
            f"Autonomous + V2V Controller started for {vehicle_id.upper()} | "
            f"{len(self.waypoints)} waypoints | Initial Target Speed: {self.target_speed:.2f} m/s ({self.target_speed * 3.6:.1f} km/h)"
        )

    # ------------------------------------------------------------------ callbacks

    def gps_callback(self, msg: NavSatFix):
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.x = (msg.longitude - 85.3333) * 103275.0
            self.y = (msg.latitude - 21.9167) * 111320.0
            self.z = msg.altitude - 450.0
            self.gps_received = True
            self.odom_received = True

    def odom_callback(self, msg: Odometry):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        oz = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        odom_yaw = math.atan2(siny, cosy)

        sx, sy, sz, syaw = SPAWN_POSES.get(self.vehicle_id, (0.0, 0.0, 0.0, 0.0))
        cos_s = math.cos(syaw)
        sin_s = math.sin(syaw)

        if not self.gps_received:
            self.x = sx + (ox * cos_s - oy * sin_s)
            self.y = sy + (ox * sin_s + oy * cos_s)
            self.z = sz + oz
        self.yaw = self._normalize_angle(odom_yaw + syaw)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx**2 + vy**2)
        self.odom_received = True

    def risk_callback(self, msg: String):
        self.risk_level = msg.data.strip()

    def set_target_speed_callback(self, msg: Float32):
        new_speed = max(0.0, min(5.56, float(msg.data)))  # 0 to 20 km/h (5.56 m/s)
        self.target_speed = new_speed
        self.get_logger().info(
            f"{self.vehicle_id.upper()} Operator set target speed: {self.target_speed:.2f} m/s ({self.target_speed * 3.6:.1f} km/h)"
        )
        self.publish_target_speed()

    def fleet_v2v_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            vid = data.get("vehicle_id")
            if vid and vid != self.vehicle_id:
                self.peer_states[vid] = data
        except Exception:
            pass

    # ------------------------------------------------------------------ V2V Broadcast & Telemetry

    def broadcast_v2v_state(self):
        if not self.odom_received:
            return

        state_packet = {
            "vehicle_id": self.vehicle_id,
            "timestamp": time.time(),
            "state": self.v2v_state,
            "phase": self.cycle_phase,
            "conflict_zone": self.active_conflict_zone,
            "claim_time": self.conflict_claim_time,
            "zone_distance": round(self.current_zone_dist, 2) if self.current_zone_dist is not None else None,
            "x": round(self.x, 2),
            "y": round(self.y, 2),
            "z": round(self.z, 2),
            "yaw": round(self.yaw, 3),
            "speed_mps": round(self.current_speed, 2),
            "speed_kmh": round(self.current_speed * 3.6, 1),
            "target_speed_mps": round(self.target_speed, 2),
            "target_speed_kmh": round(self.target_speed * 3.6, 1),
            "risk_level": self.risk_level,
            "wp_index": self.wp_index,
            "leader_id": self.leader_id,
            "dist_to_leader_m": round(self.dist_to_leader, 1) if self.dist_to_leader else None,
            "lateral_offset_m": round(self.lateral_offset, 2),
        }

        s_msg = String()
        s_msg.data = json.dumps(state_packet)
        self.pub_fleet_v2v.publish(s_msg)

        # Dedicated per-vehicle topic: /{vehicle_id}/v2v/state
        v2v_s = String()
        v2v_s.data = str(self.v2v_state)
        self.pub_vehicle_v2v_state.publish(v2v_s)

        # Also publish target speed & nav status
        self.publish_target_speed()

    def publish_target_speed(self):
        sp_msg = Float32()
        sp_msg.data = float(self.target_speed)
        self.pub_target_speed.publish(sp_msg)

        wp_coords = list(self.current_wp_target) if self.current_wp_target else [0.0, 0.0, 0.0]
        nav_info = {
            "vehicle_id": self.vehicle_id,
            "status": self.v2v_state if self.dwell_until is None else f"DWELLING ({self.cycle_phase})",
            "autonomous_status": "ACTIVE",
            "cycle_phase": self.cycle_phase,
            "v2v_state": self.v2v_state,
            "route": f"PIT HAUL LOOP ({self.cycle_phase})",
            "wp_index": self.wp_index,
            "total_wps": len(self.current_route),
            "target_wp": wp_coords,
            "dist_to_wp_m": round(self.dist_to_wp, 1),
            "target_speed_mps": round(self.target_speed, 2),
            "target_speed_kmh": round(self.target_speed * 3.6, 1),
            "leader_id": self.leader_id,
            "dist_to_leader_m": round(self.dist_to_leader, 1) if self.dist_to_leader else None,
            "lateral_offset_m": round(self.lateral_offset, 2),
        }
        n_msg = String()
        n_msg.data = json.dumps(nav_info)
        self.pub_nav_status.publish(n_msg)

    # ------------------------------------------------------------------ V2V Coordination Logic

    def _log_v2v(self, msg: str):
        self.get_logger().info(f"[V2V] {msg}")

    def evaluate_v2v_coordination(self):
        """
        Evaluates peer positions, conflict zones, and headings.
        Determines:
          - Deterministic right-of-way arbitration at conflict zones
          - Safe headway tracking behind leader vehicles along haul routes
          - Immediate zero-velocity yield on waiting states
        """
        now = time.time()
        closest_leader_dist = float("inf")
        closest_leader = None
        closest_is_oncoming = False

        # Clean stale peer messages (> 3.0s)
        active_peers = {}
        for pid, pdata in self.peer_states.items():
            if now - pdata.get("timestamp", 0) < 3.0:
                active_peers[pid] = pdata

        # ============================================================
        # 1. DETERMINISTIC CONFLICT ZONE ARBITRATION
        # ============================================================
        my_zone_key = None
        my_zone_dist = None
        for zk, zinfo in CONFLICT_ZONES.items():
            cx, cy = zinfo["center"]
            d = math.sqrt((self.x - cx)**2 + (self.y - cy)**2)
            if d <= zinfo["approach_dist"]:
                my_zone_key = zk
                my_zone_dist = d
                break

        self.current_zone_dist = my_zone_dist

        if my_zone_key:
            zinfo = CONFLICT_ZONES[my_zone_key]

            # Transition into approach
            if self.active_conflict_zone != my_zone_key:
                self.active_conflict_zone = my_zone_key
                self.conflict_claim_time = now
                self.conflict_inside = False
                self._log_v2v(f"{self.vehicle_id.upper()} -> APPROACHING_CONFLICT ({my_zone_key})")
                self._log_v2v(f"CONFLICT DETECTED at {my_zone_key}")

            if my_zone_dist <= zinfo["radius"]:
                self.conflict_inside = True

            # Scan peers approaching or occupying this specific zone
            zone_peers_inside = []
            zone_peers_approaching = []

            for pid, pdata in active_peers.items():
                px, py = pdata.get("x", 0.0), pdata.get("y", 0.0)
                cx, cy = zinfo["center"]
                pd = math.sqrt((px - cx)**2 + (py - cy)**2)
                p_state = pdata.get("state", "")
                p_zone = pdata.get("conflict_zone")

                if pd <= (zinfo["radius"] + 1.2) or (p_zone == my_zone_key and "CROSSING" in p_state):
                    zone_peers_inside.append((pid, pdata, pd))
                elif pd <= zinfo["approach_dist"] or (p_zone == my_zone_key and "APPROACHING" in p_state):
                    p_claim = pdata.get("claim_time") or pdata.get("timestamp") or float("inf")
                    zone_peers_approaching.append((pid, pdata, pd, p_claim))

            # Rule 1: Active crossing peer inside zone has exclusive right-of-way
            if zone_peers_inside and not self.conflict_inside:
                blocking_pid = zone_peers_inside[0][0]
                new_state = f"WAITING — {blocking_pid.upper()} HAS PRIORITY"
                if self.v2v_state != new_state:
                    self.v2v_state = new_state
                    self._log_v2v(f"{self.vehicle_id.upper()} -> WAITING_FOR_V2V (linear.x=0.0, angular.z=0.0)")
                return 0.0, 0.0

            # Rule 2: Multiple approaching peers -> FIFO arrival claim time (tie-break by vehicle_id)
            if zone_peers_approaching and not self.conflict_inside:
                peer_has_priority = False
                prio_pid = None
                my_claim = self.conflict_claim_time if self.conflict_claim_time is not None else now

                for pid, pdata, pd, p_claim in zone_peers_approaching:
                    if (my_claim - p_claim) > 0.2:
                        peer_has_priority = True
                        prio_pid = pid
                        break
                    elif abs(my_claim - p_claim) <= 0.2 and pid < self.vehicle_id:
                        peer_has_priority = True
                        prio_pid = pid
                        break

                if peer_has_priority:
                    new_state = f"WAITING — {prio_pid.upper()} HAS PRIORITY"
                    if self.v2v_state != new_state:
                        self.v2v_state = new_state
                        self._log_v2v(f"{self.vehicle_id.upper()} -> WAITING_FOR_V2V (linear.x=0.0, angular.z=0.0)")
                    return 0.0, 0.0

            # Rule 3: This vehicle has right-of-way!
            if "WAITING" in self.v2v_state:
                self._log_v2v(f"{self.vehicle_id.upper()} -> RELEASED / RESUMING")
                self.v2v_state = "RELEASED / RESUMING"
                self.resuming_until = now + 1.5

            if self.conflict_inside:
                if self.v2v_state != "CROSSING":
                    self.v2v_state = "CROSSING"
                    self._log_v2v(f"{self.vehicle_id.upper()} -> CROSSING")
            else:
                if now >= self.resuming_until and self.v2v_state not in ["PRIORITY / GO", "CROSSING"]:
                    self.v2v_state = "PRIORITY / GO"
                    self._log_v2v(f"{self.vehicle_id.upper()} -> PRIORITY / GO")

            self.lateral_offset = 0.0
            return self.target_speed, 0.0

        else:
            # Self is outside all conflict zones
            if self.active_conflict_zone is not None:
                self._log_v2v(f"{self.vehicle_id.upper()} -> CLEAR")
                self.active_conflict_zone = None
                self.conflict_claim_time = None
                self.conflict_inside = False
                self.v2v_state = "CLEAR"
                self.clear_until = now + 1.2

        # ============================================================
        # 2. HEADWAY & PEER FOLLOWING OUTSIDE CONFLICT ZONES
        # ============================================================
        for pid, pdata in active_peers.items():
            px = pdata["x"]
            py = pdata["y"]
            dx = px - self.x
            dy = py - self.y
            dist = math.sqrt(dx**2 + dy**2)

            if dist > V2V_DETECTION_DIST_M:
                continue

            cos_h = math.cos(self.yaw)
            sin_h = math.sin(self.yaw)
            longitudinal = dx * cos_h + dy * sin_h
            lateral = -dx * sin_h + dy * cos_h

            peer_yaw = pdata.get("yaw", 0.0)
            yaw_diff = abs(self._normalize_angle(peer_yaw - self.yaw))
            is_oncoming = (yaw_diff > math.radians(100.0))

            # Dual-lane passing clearance: If oncoming in adjacent lane, safe
            if is_oncoming and (abs(lateral) > 1.8 or abs(px - self.x) > 2.5):
                continue

            # Check if peer is ahead in our travel corridor
            if longitudinal > 0.5 and abs(lateral) < 3.8 and dist < closest_leader_dist:
                closest_leader_dist = dist
                closest_leader = pdata
                closest_is_oncoming = is_oncoming

        if closest_leader:
            self.leader_id = closest_leader["vehicle_id"]
            self.dist_to_leader = closest_leader_dist
        else:
            self.leader_id = None
            self.dist_to_leader = None

        # Reset EMERGENCY_WAIT if cleared
        if self.v2v_state == "EMERGENCY_WAIT":
            if not closest_leader or closest_is_oncoming or closest_leader.get("risk_level") != "CRITICAL" or closest_leader_dist > 14.0:
                self.v2v_state = "NORMAL"

        # Oncoming encroachment in narrow choke-point
        if closest_leader and closest_is_oncoming:
            my_priority = (self.vehicle_id < closest_leader["vehicle_id"])
            if not my_priority:
                self.v2v_state = "YIELDING"
                return (0.0 if closest_leader_dist < 10.0 else min(self.target_speed, 0.8)), 0.0
            else:
                self.v2v_state = "NORMAL"
                return self.target_speed, 0.0

        # Critical stopped leader ahead
        if closest_leader and not closest_is_oncoming and closest_leader.get("risk_level") == "CRITICAL" and closest_leader_dist < 12.0:
            self.v2v_state = "EMERGENCY_WAIT"
            return 0.0, 0.0

        # Leader headway regulation
        if closest_leader:
            leader_spd = closest_leader.get("speed_mps", 0.0)
            is_leader_stopped = (leader_spd < 0.2)

            if is_leader_stopped:
                if closest_leader_dist <= V2V_STOP_BUFFER_M:
                    self.v2v_state = "WAITING"
                    return 0.0, 0.0
                elif closest_leader_dist <= V2V_FOLLOW_SAFE_DIST_M:
                    self.v2v_state = "FOLLOWING"
                    return min(self.target_speed, 0.8), 0.0
                else:
                    self.v2v_state = "NORMAL"
                    return self.target_speed, 0.0

            if closest_leader_dist < V2V_STOP_BUFFER_M:
                self.v2v_state = "WAITING"
                return 0.0, 0.0
            elif closest_leader_dist < V2V_FOLLOW_SAFE_DIST_M:
                self.v2v_state = "FOLLOWING"
                headway_factor = (closest_leader_dist - V2V_STOP_BUFFER_M) / (V2V_FOLLOW_SAFE_DIST_M - V2V_STOP_BUFFER_M)
                target_lead_speed = leader_spd * (0.4 + 0.6 * max(0.0, min(1.0, headway_factor)))
                return min(self.target_speed, max(0.5, target_lead_speed)), 0.0

        # Normal cruising
        if now < self.clear_until:
            self.v2v_state = "CLEAR"
        elif now < self.resuming_until:
            self.v2v_state = "RELEASED / RESUMING"
        elif self.v2v_state not in ["NORMAL"]:
            self.v2v_state = "NORMAL"

        self.lateral_offset = 0.0
        return self.target_speed, 0.0

    # ------------------------------------------------------------------ Control Loop

    def control_loop(self):
        if not self.odom_received:
            return

        # Check dwell pause (Loading or Dumping Pad)
        if self.dwell_until is not None:
            if time.time() < self.dwell_until:
                self._publish_stop()
                return
            else:
                self.dwell_until = None
                self.get_logger().info(f"{self.vehicle_id.upper()} Dwell completed. Resuming haul cycle.")

        # Check waypoint bounds and wrap around for endless continuous haulage
        if self.wp_index >= len(self.current_route):
            if self.vehicle_id == "hemm_01":
                self.wp_index = next((i for i, w in enumerate(self.current_route) if abs(w[0] - 30.0) < 3.0 and abs(w[1] - 85.0) < 3.0), 3)
            else:
                self.wp_index = 0
            self.get_logger().info(f"{self.vehicle_id.upper()} Completed circuit. Restarting continuous cycle at WP{self.wp_index}.")

        wp = self.current_route[self.wp_index]
        self.current_wp_target = wp
        tx, ty, tz = wp

        # 2D distance to raw waypoint
        dx = tx - self.x
        dy = ty - self.y
        dist_2d = math.sqrt(dx**2 + dy**2)
        self.dist_to_wp = dist_2d

        # Check if reached waypoint
        heading_to_wp = math.atan2(dy, dx)
        angle_to_wp = abs(self._normalize_angle(heading_to_wp - self.yaw))
        reached = (dist_2d < 3.5) or (dist_2d < 4.8 and angle_to_wp > math.radians(100.0))

        if reached:
            self.get_logger().info(
                f"{self.vehicle_id.upper()} Reached WP{self.wp_index} ({tx:.1f}, {ty:.1f}, {tz:.1f}) | Next WP"
            )

            # Check if this is the Loading Zone
            if abs(tx - 0.0) < 3.0 and abs(ty - 2.0) < 3.0:
                self.cycle_phase = "LOADING"
                self.dwell_until = time.time() + DWELL_TIME_S
                self.get_logger().info(f"{self.vehicle_id.upper()} === AT LOADING ZONE (Dwelling {DWELL_TIME_S}s) ===")
                self._publish_stop()
                self.wp_index += 1
                return

            # Check if this is the Dumping Zone (Bench 1 x=30, y=85.0)
            if abs(tx - 30.0) <= 2.0 and abs(ty - 85.0) <= 2.0:
                self.cycle_phase = "DUMPING"
                self.dwell_until = time.time() + DWELL_TIME_S
                self.get_logger().info(f"{self.vehicle_id.upper()} === AT DUMPING ZONE (Dwelling {DWELL_TIME_S}s) ===")
                self._publish_stop()
                self.wp_index += 1
                return

            # Advance to next waypoint
            self.wp_index += 1
            if self.wp_index < len(self.current_route):
                next_wp = self.current_route[self.wp_index]
                if 54.0 <= next_wp[1] <= 82.0 and abs(next_wp[0]) <= 8.0:
                    self.cycle_phase = "RAMP 1"
                elif 20.0 <= next_wp[1] <= 50.0 and abs(next_wp[0]) <= 8.0:
                    self.cycle_phase = "RAMP 2"
                elif next_wp[1] >= 82.0 and next_wp[0] < 10.0:
                    self.cycle_phase = "RETURNING"
                else:
                    self.cycle_phase = "HAULING"
            return

        # Continuous cycle phase tracking based on active position
        if self.dwell_until is None:
            if 54.0 <= self.y <= 82.0 and abs(self.x) <= 8.0:
                self.cycle_phase = "RAMP 1"
            elif 20.0 <= self.y <= 50.0 and abs(self.x) <= 8.0:
                self.cycle_phase = "RAMP 2"
            elif self.y >= 82.0:
                if math.cos(self.yaw) < -0.2:
                    self.cycle_phase = "RETURNING"
                else:
                    self.cycle_phase = "HAULING"
            else:
                self.cycle_phase = "HAULING"

        # ============================================================
        # SPEED & STEERING ARBITRATION ARCHITECTURE:
        #   1. AUTONOMOUS ROUTE (Target heading along lane corridor)
        #   2. ANTI-SKID 3-TIER STEERING & VELOCITY ENVELOPE
        #   3. V2V COORDINATION OVERRIDE (Right-of-Way / Waiting / Headway)
        #   4. SAFETY CEILING (LiDAR Risk Level Governor)
        #   5. TRUCK ACTUATION (/cmd_vel)
        # ============================================================

        # V2V Fleet Coordination evaluation
        v2v_speed_cap, lateral_shift = self.evaluate_v2v_coordination()

        target_x = tx
        target_y = ty  # Strict lane corridor tracking, no lateral shifting

        t_dx = target_x - self.x
        t_dy = target_y - self.y

        desired_yaw = math.atan2(t_dy, t_dx)
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        cross_prod = cos_y * t_dy - sin_y * t_dx
        dot_prod = cos_y * t_dx + sin_y * t_dy
        heading_error = math.atan2(cross_prod, dot_prod)
        heading_abs = abs(heading_error)

        # ------------------------------------------------------------
        # ANTI-SKID 3-TIER HEADING ERROR CONTROL:
        # Tier 3 (Large heading error > 28 deg / 0.488 rad):
        #   STOP forward motion completely (linear.x = 0.0 m/s).
        #   Rotate in place smoothly (0.25 - 0.45 rad/s).
        #   Hysteresis: resume forward motion only once heading error <= 18 deg.
        # Tier 2 (Medium heading error 10 - 28 deg):
        #   Reduce forward speed smoothly.
        #   Curvature steering capped <= 0.40 rad/s.
        # Tier 1 (Small heading error <= 10 deg / 0.175 rad):
        #   Full cruising speed.
        #   Gentle steering capped <= 0.25 rad/s.
        # ------------------------------------------------------------
        if heading_abs > math.radians(28.0):
            self.aligning_in_place = True
        elif heading_abs <= math.radians(18.0):
            self.aligning_in_place = False

        if self.aligning_in_place:
            route_speed = 0.0  # STOP forward motion completely
            turn_dir = 1.0 if heading_error > 0.0 else -1.0
            turn_rate = min(0.45, max(0.25, 0.9 * heading_abs))
            angular_z = turn_dir * turn_rate
        elif heading_abs > math.radians(10.0):
            err_ratio = (heading_abs - math.radians(10.0)) / math.radians(18.0)
            route_speed = self.target_speed * (1.0 - 0.6 * min(1.0, max(0.0, err_ratio)))
            raw_angular = 0.8 * heading_error
            angular_z = max(-0.40, min(0.40, raw_angular))
        else:
            route_speed = self.target_speed
            raw_angular = 0.8 * heading_error
            angular_z = max(-0.25, min(0.25, raw_angular))

        # Dynamic Velocity Envelope: |angular_z| <= 1.2 / (1.0 + 1.8 * speed)
        env_limit = 1.2 / (1.0 + 1.8 * max(route_speed, self.current_speed))
        angular_z = max(-env_limit, min(env_limit, angular_z))

        # Apply V2V speed cap
        coordinated_speed = min(route_speed, v2v_speed_cap)

        # Safety Limit (LiDAR Governor) - Strictly non-overridable veto authority
        safety_ceiling = SPEED_BY_RISK.get(self.risk_level, 0.0)
        final_speed = min(coordinated_speed, safety_ceiling)

        # Enforce complete stop for Safety CRITICAL or V2V Waiting states
        if (safety_ceiling <= 0.0 or 
            self.risk_level == "CRITICAL" or 
            self.v2v_state.startswith("WAITING") or 
            self.v2v_state in ["EMERGENCY_WAIT", "YIELDING"]):
            final_speed = 0.0
            angular_z = 0.0

        self._publish_cmd(final_speed, angular_z)

    # ------------------------------------------------------------------ helpers

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:   angle -= 2 * math.pi
        while angle < -math.pi:  angle += 2 * math.pi
        return angle

    def _publish_cmd(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def _publish_stop(self):
        self._publish_cmd(0.0, 0.0)


# ================================================================== main

def main(argv=None):
    parser = argparse.ArgumentParser(description="HEMM Autonomous + V2V Controller")
    parser.add_argument("--vehicle-id", required=True,
                        choices=["hemm_01", "hemm_02", "hemm_03"],
                        help="Vehicle namespace identifier")
    parser.add_argument("--speed", type=float, default=2.2,
                        help="Target cruising speed setting in m/s (default: 2.2 m/s)")
    args, ros_args = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    rclpy.init(args=ros_args)
    node = HEMMAutonomousController(args.vehicle_id, target_speed=args.speed)
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
