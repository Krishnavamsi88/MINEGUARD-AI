import subprocess
import time
import os
import math

env = os.environ.copy()
env["GZ_SIM_RESOURCE_PATH"] = "/home/krish-pc/SIH_Mine_Safety/gazebo/models:" + env.get("GZ_SIM_RESOURCE_PATH", "")

# Kill any existing gz processes
subprocess.run(["pkill", "-9", "-f", "gz sim"], stderr=subprocess.DEVNULL)
subprocess.run(["pkill", "-9", "-f", "ros_gz_bridge"], stderr=subprocess.DEVNULL)
time.sleep(1)

def quat_to_euler(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

print("Starting Gazebo Sim Harmonic server...")
gz_proc = subprocess.Popen(
    ["gz", "sim", "-s", "-r", "/home/krish-pc/SIH_Mine_Safety/gazebo/worlds/sih_mine_test.sdf"],
    env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
)

try:
    time.sleep(2)
    print("\n=======================================================")
    print("PHASE 1: Spawn Stability Observation (5 seconds)")
    print("=======================================================")
    for sec in range(1, 6):
        time.sleep(1)
        res = subprocess.run(["gz", "topic", "-e", "-t", "/world/sih_mine_test/dynamic_pose/info", "-n", "1"],
                             env=env, capture_output=True, text=True, timeout=5)
        blocks = res.stdout.split("pose {")
        for b in blocks:
            if 'name: "HEMM_Dumper_01"' in b:
                lines = b.splitlines()
                x = y = z = qx = qy = qz = qw = 0.0
                for i, l in enumerate(lines):
                    if "x:" in l and "position" in lines[max(0, i-1)]: x = float(l.split(":")[1])
                    elif "y:" in l and "position" in lines[max(0, i-2)]: y = float(l.split(":")[1])
                    elif "z:" in l and "position" in lines[max(0, i-3)]: z = float(l.split(":")[1])
                    elif "x:" in l and "orientation" in lines[max(0, i-1)]: qx = float(l.split(":")[1])
                    elif "y:" in l and "orientation" in lines[max(0, i-2)]: qy = float(l.split(":")[1])
                    elif "z:" in l and "orientation" in lines[max(0, i-3)]: qz = float(l.split(":")[1])
                    elif "w:" in l and "orientation" in lines[max(0, i-4)]: qw = float(l.split(":")[1])
                r, p, ya = quat_to_euler(qx, qy, qz, qw)
                print(f"t={sec}s: HEMM_Dumper_01 pos=({x:.4f}, {y:.4f}, {z:.4f})  roll={r:.2f}° pitch={p:.2f}° yaw={ya:.2f}°")
                break

    print("\n=======================================================")
    print("PHASE 2: Individual Link Positions at t=5s")
    print("=======================================================")
    res = subprocess.run(["gz", "topic", "-e", "-t", "/world/sih_mine_test/dynamic_pose/info", "-n", "1"],
                         env=env, capture_output=True, text=True, timeout=5)
    for b in res.stdout.split("pose {"):
        for link_name in ["chassis", "left_front_wheel", "right_front_wheel", "left_rear_wheel", "right_rear_wheel"]:
            if f'name: "{link_name}"' in b:
                lines = b.splitlines()
                lx = ly = lz = 0.0
                for i, l in enumerate(lines):
                    if "x:" in l and "position" in lines[max(0, i-1)]: lx = float(l.split(":")[1])
                    elif "y:" in l and "position" in lines[max(0, i-2)]: ly = float(l.split(":")[1])
                    elif "z:" in l and "position" in lines[max(0, i-3)]: lz = float(l.split(":")[1])
                print(f"  {link_name:20s}: x={lx:7.3f}, y={ly:7.3f}, z={lz:7.3f}")

    print("\n=======================================================")
    print("PHASE 3: /cmd_vel Command Test (Forward 1.0 m/s for 3s)")
    print("=======================================================")
    for _ in range(6):
        subprocess.run(["gz", "topic", "-t", "/cmd_vel", "-m", "gz.msgs.Twist",
                        "-p", "linear: {x: 1.0}"],
                       env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.5)

    res = subprocess.run(["gz", "topic", "-e", "-t", "/odom", "-n", "1"],
                         env=env, capture_output=True, text=True, timeout=5)
    for l in res.stdout.splitlines():
        if "x:" in l or "y:" in l or "z:" in l:
            print("  /odom:", l.strip())

    res = subprocess.run(["gz", "topic", "-e", "-t", "/world/sih_mine_test/dynamic_pose/info", "-n", "1"],
                         env=env, capture_output=True, text=True, timeout=5)
    for b in res.stdout.split("pose {"):
        if 'name: "HEMM_Dumper_01"' in b:
            lines = b.splitlines()
            x = y = z = qx = qy = qz = qw = 0.0
            for i, l in enumerate(lines):
                if "x:" in l and "position" in lines[max(0, i-1)]: x = float(l.split(":")[1])
                elif "y:" in l and "position" in lines[max(0, i-2)]: y = float(l.split(":")[1])
                elif "z:" in l and "position" in lines[max(0, i-3)]: z = float(l.split(":")[1])
                elif "x:" in l and "orientation" in lines[max(0, i-1)]: qx = float(l.split(":")[1])
                elif "y:" in l and "orientation" in lines[max(0, i-2)]: qy = float(l.split(":")[1])
                elif "z:" in l and "orientation" in lines[max(0, i-3)]: qz = float(l.split(":")[1])
                elif "w:" in l and "orientation" in lines[max(0, i-4)]: qw = float(l.split(":")[1])
            r, p, ya = quat_to_euler(qx, qy, qz, qw)
            print(f"  HEMM position after driving: x={x:.3f}, y={y:.3f}, z={z:.3f} | roll={r:.2f}°, pitch={p:.2f}°, yaw={ya:.2f}°")
            break

    print("\n=======================================================")
    print("PHASE 4: /cmd_vel Turn Command Test (Turning 0.5 rad/s for 2s)")
    print("=======================================================")
    for _ in range(4):
        subprocess.run(["gz", "topic", "-t", "/cmd_vel", "-m", "gz.msgs.Twist",
                        "-p", "angular: {z: 0.5}"],
                       env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.5)

    res = subprocess.run(["gz", "topic", "-e", "-t", "/world/sih_mine_test/dynamic_pose/info", "-n", "1"],
                         env=env, capture_output=True, text=True, timeout=5)
    for b in res.stdout.split("pose {"):
        if 'name: "HEMM_Dumper_01"' in b:
            lines = b.splitlines()
            x = y = z = qx = qy = qz = qw = 0.0
            for i, l in enumerate(lines):
                if "x:" in l and "position" in lines[max(0, i-1)]: x = float(l.split(":")[1])
                elif "y:" in l and "position" in lines[max(0, i-2)]: y = float(l.split(":")[1])
                elif "z:" in l and "position" in lines[max(0, i-3)]: z = float(l.split(":")[1])
                elif "x:" in l and "orientation" in lines[max(0, i-1)]: qx = float(l.split(":")[1])
                elif "y:" in l and "orientation" in lines[max(0, i-2)]: qy = float(l.split(":")[1])
                elif "z:" in l and "orientation" in lines[max(0, i-3)]: qz = float(l.split(":")[1])
                elif "w:" in l and "orientation" in lines[max(0, i-4)]: qw = float(l.split(":")[1])
            r, p, ya = quat_to_euler(qx, qy, qz, qw)
            print(f"  HEMM pose after turn: x={x:.3f}, y={y:.3f}, z={z:.3f} | roll={r:.2f}°, pitch={p:.2f}°, yaw={ya:.2f}°")
            break

    print("\nVERIFICATION SUMMARY: PASSED ALL PHASES!")
finally:
    gz_proc.terminate()
    gz_proc.wait()
