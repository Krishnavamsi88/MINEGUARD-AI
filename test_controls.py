"""
Detailed control and motion test for HEMM in sih_mine_world.sdf
Tests:
1. 10s stationary spawn stability
2. Forward motion
3. Reverse motion
4. Turning motion (curved drive and spin)
5. /odom response
"""
import subprocess, time, os, math

env = os.environ.copy()
env["GZ_SIM_RESOURCE_PATH"] = (
    "/home/krish-pc/SIH_Mine_Safety/gazebo/models:"
    + env.get("GZ_SIM_RESOURCE_PATH", "")
)
WORLD = "/home/krish-pc/SIH_Mine_Safety/gazebo/worlds/sih_mine_world.sdf"
WORLD_NAME = "sih_mine_world"

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

def get_odom():
    res = subprocess.run(
        ["gz", "topic", "-e", "-t", "/odom", "-n", "1"],
        env=env, capture_output=True, text=True, timeout=5)
    return res.stdout

def cmd_vel(lx=0.0, az=0.0, duration=2.0, rate=5):
    steps = int(duration * rate)
    for _ in range(steps):
        subprocess.run(
            ["gz", "topic", "-t", "/cmd_vel", "-m", "gz.msgs.Twist",
             "-p", f"linear: {{x: {lx}}}, angular: {{z: {az}}}"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.0 / rate)

# Kill previous
subprocess.run(["pkill", "-9", "-f", "gz sim"], stderr=subprocess.DEVNULL)
time.sleep(1)

print("Starting Gazebo Sim Harmonic with SIH Mine World...", flush=True)
gz = subprocess.Popen(
    ["gz", "sim", "-s", "-r", WORLD],
    env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

try:
    time.sleep(4)

    print("\n--- TEST 1: 10-Second Stationary Stability ---", flush=True)
    for sec in range(1, 11):
        time.sleep(1)
        p = get_pose()
        if p:
            print(f"  t={sec:2d}s: pos=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) | roll={p[3]:.2f}°, pitch={p[4]:.2f}°, yaw={p[5]:.2f}°", flush=True)

    print("\n--- TEST 2: Forward Motion (linear.x = 1.5 m/s, 3s) ---", flush=True)
    p_pre_fwd = get_pose()
    cmd_vel(lx=1.5, az=0.0, duration=3.0)
    p_post_fwd = get_pose()
    dist_fwd = math.sqrt((p_post_fwd[0] - p_pre_fwd[0])**2 + (p_post_fwd[1] - p_pre_fwd[1])**2)
    print(f"  Before: ({p_pre_fwd[0]:.3f}, {p_pre_fwd[1]:.3f}) -> After: ({p_post_fwd[0]:.3f}, {p_post_fwd[1]:.3f})", flush=True)
    print(f"  Forward displacement: {dist_fwd:.3f} m (expected ~4.5m)", flush=True)

    print("\n--- TEST 3: Reverse Motion (linear.x = -1.5 m/s, 3s) ---", flush=True)
    p_pre_rev = get_pose()
    cmd_vel(lx=-1.5, az=0.0, duration=3.0)
    p_post_rev = get_pose()
    dist_rev = math.sqrt((p_post_rev[0] - p_pre_rev[0])**2 + (p_post_rev[1] - p_pre_rev[1])**2)
    print(f"  Before: ({p_pre_rev[0]:.3f}, {p_pre_rev[1]:.3f}) -> After: ({p_post_rev[0]:.3f}, {p_post_rev[1]:.3f})", flush=True)
    print(f"  Reverse displacement: {dist_rev:.3f} m (expected ~4.5m)", flush=True)

    print("\n--- TEST 4: Curve Turning Motion (linear.x = 1.0, angular.z = 0.5, 4s) ---", flush=True)
    p_pre_curve = get_pose()
    cmd_vel(lx=1.0, az=0.5, duration=4.0)
    p_post_curve = get_pose()
    dyaw_curve = p_post_curve[5] - p_pre_curve[5]
    print(f"  Before yaw: {p_pre_curve[5]:.2f}° -> After yaw: {p_post_curve[5]:.2f}° | Change: {dyaw_curve:.2f}°", flush=True)
    print(f"  Position: ({p_post_curve[0]:.3f}, {p_post_curve[1]:.3f})", flush=True)

    print("\n--- TEST 5: Spin Turn (linear.x = 0.0, angular.z = 0.5, 3s) ---", flush=True)
    p_pre_spin = get_pose()
    cmd_vel(lx=0.0, az=0.5, duration=3.0)
    p_post_spin = get_pose()
    dyaw_spin = p_post_spin[5] - p_pre_spin[5]
    print(f"  Before yaw: {p_pre_spin[5]:.2f}° -> After yaw: {p_post_spin[5]:.2f}° | Change: {dyaw_spin:.2f}°", flush=True)

    print("\n--- TEST 6: /odom Reading ---", flush=True)
    odom_txt = get_odom()
    for line in odom_txt.splitlines():
        if any(k in line for k in ["frame_id", "position", "orientation", "linear", "angular", "x:", "y:", "z:", "w:"]):
            print(" ", line.strip(), flush=True)

    print("\nAll tests finished.", flush=True)
finally:
    gz.terminate()
    gz.wait()
