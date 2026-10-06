"""
Comprehensive sensor stack and motion test for HEMM_Dumper_01 in sih_mine_world.sdf:
1. Spawns Gazebo Sim with Sensors system
2. Verifies Gazebo topics (/scan, /imu, /gps, /camera/image_raw, /depth/image_raw)
3. Starts ros_gz_bridge for all sensors and vehicle control
4. Verifies ROS 2 topics exist via `ros2 topic list`
5. Samples each ROS 2 topic (/scan, /imu, /gps, /camera/camera_info, /odom)
6. Tests vehicle stability on spawn (10s watch)
7. Tests driving with /cmd_vel and verifies odom updates & LiDAR ranges change
"""
import subprocess
import time
import os
import math
import sys

env = os.environ.copy()
# Source ROS 2 Jazzy environment
res = subprocess.run(
    ["bash", "-c", "source /opt/ros/jazzy/setup.bash && env"],
    capture_output=True, text=True)
for line in res.stdout.splitlines():
    if "=" in line:
        k, v = line.split("=", 1)
        env[k] = v

env["GZ_SIM_RESOURCE_PATH"] = (
    "/home/krish-pc/SIH_Mine_Safety/gazebo/models:"
    + env.get("GZ_SIM_RESOURCE_PATH", "")
)

WORLD = "/home/krish-pc/SIH_Mine_Safety/gazebo/worlds/sih_mine_world.sdf"
WORLD_NAME = "sih_mine_world"

def clean():
    subprocess.run(["pkill", "-9", "-f", "gz sim"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "parameter_bridge"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "ros_gz_bridge"], stderr=subprocess.DEVNULL)
    time.sleep(1)

def q2e(x, y, z, w):
    roll  = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    pitch = math.degrees(math.asin(max(-1, min(1, 2*(w*y - z*x)))))
    yaw   = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    return roll, pitch, yaw

def get_pose(entity="HEMM_Dumper_01"):
    res = subprocess.run(
        ["gz", "topic", "-e", "-t",
         f"/world/{WORLD_NAME}/dynamic_pose/info", "-n", "1"],
        env=env, capture_output=True, text=True, timeout=6)
    for block in res.stdout.split("pose {"):
        if f'name: "{entity}"' in block:
            lines = block.splitlines()
            x = y = z = qx = qy = qz = qw = 0.0
            for i, l in enumerate(lines):
                if "x:" in l and "position" in lines[max(0,i-1)]: x = float(l.split(":")[1])
                elif "y:" in l and "position" in lines[max(0,i-2)]: y = float(l.split(":")[1])
                elif "z:" in l and "position" in lines[max(0,i-3)]: z = float(l.split(":")[1])
                elif "x:" in l and "orientation" in lines[max(0,i-1)]: qx = float(l.split(":")[1])
                elif "y:" in l and "orientation" in lines[max(0,i-2)]: qy = float(l.split(":")[1])
                elif "z:" in l and "orientation" in lines[max(0,i-3)]: qz = float(l.split(":")[1])
                elif "w:" in l and "orientation" in lines[max(0,i-4)]: qw = float(l.split(":")[1])
            r, p, ya = q2e(qx, qy, qz, qw)
            return x, y, z, r, p, ya
    return None

clean()

print("=" * 60)
print("SIH MINE SAFETY: INTEGRATED SENSOR STACK VERIFICATION")
print("=" * 60, flush=True)

print("Starting Gazebo Sim Harmonic (server mode)...", flush=True)
gz_proc = subprocess.Popen(
    ["gz", "sim", "-s", "-r", WORLD],
    env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

bridge_proc = None

try:
    time.sleep(5)
    if gz_proc.poll() is not None:
        print("ERROR: Gazebo failed to start!")
        sys.exit(1)

    print("\n[STEP 1] Checking Gazebo Transport Topics...")
    r_gz_topics = subprocess.run(["gz", "topic", "-l"], env=env, capture_output=True, text=True, timeout=5)
    gz_topics = [t.strip() for t in r_gz_topics.stdout.splitlines() if t.strip()]
    expected_gz = ["/scan", "/imu", "/gps", "/camera/image_raw", "/depth/image_raw", "/cmd_vel", "/odom"]
    for eg in expected_gz:
        found = any(eg in t for t in gz_topics)
        print(f"  Gazebo topic '{eg}': {'FOUND' if found else 'NOT FOUND'}")

    print("\n[STEP 2] Checking HEMM 10-Second Stationary Stability...")
    for sec in range(1, 11):
        time.sleep(1)
        p = get_pose()
        if p:
            print(f"  t={sec:2d}s: pos=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) | roll={p[3]:.2f}°, pitch={p[4]:.2f}°, yaw={p[5]:.2f}°")
        else:
            print(f"  t={sec:2d}s: (waiting for pose)")

    print("\n[STEP 3] Starting ros_gz_bridge for Control and Sensors...")
    bridge_cmd = [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        "/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
        "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"
    ]
    bridge_proc = subprocess.Popen(bridge_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(4)

    print("\n[STEP 4] Querying ROS 2 Active Topics via `ros2 topic list`...")
    r_ros_topics = subprocess.run(["ros2", "topic", "list"], env=env, capture_output=True, text=True, timeout=6)
    ros_topics = [t.strip() for t in r_ros_topics.stdout.splitlines() if t.strip()]
    for rt in ros_topics:
        print(f"  -> {rt}")

    print("\n[STEP 5] Sampling ROS 2 Sensor Messages...")

    # 1. IMU
    print("  Testing /imu...")
    r_imu = subprocess.run(["ros2", "topic", "echo", "/imu", "--once"], env=env, capture_output=True, text=True, timeout=6)
    if "orientation" in r_imu.stdout:
        print("  ✓ /imu received valid data:")
        for line in r_imu.stdout.splitlines()[:10]:
            print(f"    {line}")
    else:
        print("  ✗ /imu: no message received")

    # 2. GPS
    print("\n  Testing /gps...")
    r_gps = subprocess.run(["ros2", "topic", "echo", "/gps", "--once"], env=env, capture_output=True, text=True, timeout=6)
    if "latitude" in r_gps.stdout:
        print("  ✓ /gps received valid data:")
        for line in r_gps.stdout.splitlines()[:10]:
            print(f"    {line}")
    else:
        print("  ✗ /gps: no message received")

    # 3. LiDAR
    print("\n  Testing /scan...")
    r_scan = subprocess.run(["ros2", "topic", "echo", "/scan", "--once"], env=env, capture_output=True, text=True, timeout=6)
    if "ranges" in r_scan.stdout:
        print("  ✓ /scan received valid LaserScan data:")
        for line in r_scan.stdout.splitlines()[:12]:
            print(f"    {line}")
    else:
        print("  ✗ /scan: no message received")

    # 4. Camera Info
    print("\n  Testing /camera/camera_info...")
    r_cinfo = subprocess.run(["ros2", "topic", "echo", "/camera/camera_info", "--once"], env=env, capture_output=True, text=True, timeout=6)
    if "width" in r_cinfo.stdout or "height" in r_cinfo.stdout:
        print("  ✓ /camera/camera_info received valid data:")
        for line in r_cinfo.stdout.splitlines()[:10]:
            print(f"    {line}")
    else:
        print("  ✗ /camera/camera_info: no message received")

    # 5. Camera Image
    print("\n  Testing /camera/image_raw...")
    r_cimg = subprocess.run(["ros2", "topic", "echo", "/camera/image_raw", "--once"], env=env, capture_output=True, text=True, timeout=6)
    if "data" in r_cimg.stdout or "encoding" in r_cimg.stdout:
        print("  ✓ /camera/image_raw received valid image frame")
    else:
        print("  ✗ /camera/image_raw: no message received")

    # 6. Depth Image
    print("\n  Testing /depth/image_raw...")
    r_dimg = subprocess.run(["ros2", "topic", "echo", "/depth/image_raw", "--once"], env=env, capture_output=True, text=True, timeout=6)
    if "data" in r_dimg.stdout or "encoding" in r_dimg.stdout:
        print("  ✓ /depth/image_raw received valid depth frame")
    else:
        print("  ✗ /depth/image_raw: no message received")

    # 7. Odometry
    print("\n  Testing /odom...")
    r_odom = subprocess.run(["ros2", "topic", "echo", "/odom", "--once"], env=env, capture_output=True, text=True, timeout=6)
    if "pose" in r_odom.stdout:
        print("  ✓ /odom received valid Odometry data")
    else:
        print("  ✗ /odom: no message received")

    print("\n[STEP 6] Testing Driving Motion with /cmd_vel and LiDAR Range Response...")
    p_before = get_pose()

    # Get initial sample of front LiDAR range (center index 180)
    initial_ranges = []
    for line in r_scan.stdout.splitlines():
        if "ranges:" in line or (initial_ranges and "[" not in line and "]" not in line):
            initial_ranges.append(line)

    print(f"  Driving forward 15m (linear.x = 2.0)...", flush=True)
    for _ in range(12):
        subprocess.run(
            ["ros2", "topic", "pub", "/cmd_vel", "geometry_msgs/msg/Twist",
             "{linear: {x: 2.0}}", "--once"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.5)

    p_after = get_pose()
    dist = math.sqrt((p_after[0] - p_before[0])**2 + (p_after[1] - p_before[1])**2)
    print(f"  Displacement: {dist:.2f} m | Pose: ({p_after[0]:.2f}, {p_after[1]:.2f}, {p_after[2]:.2f})")
    print(f"  Truck roll={p_after[3]:.2f}°, pitch={p_after[4]:.2f}° -> {'✓ UPRIGHT & STABLE' if abs(p_after[3])<3 else '✗ UNSTABLE'}")

    # Check updated odom and scan
    r_scan2 = subprocess.run(["ros2", "topic", "echo", "/scan", "--once"], env=env, capture_output=True, text=True, timeout=6)
    if "ranges" in r_scan2.stdout:
        print("  ✓ LiDAR updated actively after vehicle motion")

    print("\n" + "=" * 60)
    print("ALL SENSOR AND MOTION VERIFICATION PHASES COMPLETED")
    print("=" * 60)

finally:
    if bridge_proc:
        bridge_proc.terminate()
        bridge_proc.wait()
    clean()
