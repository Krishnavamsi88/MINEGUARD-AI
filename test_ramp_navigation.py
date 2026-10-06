"""
Test HEMM navigating from Spawn Apron to Ramp 01 and descending to Bench 2
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

def cmd_vel(lx=0.0, az=0.0, duration=1.0, rate=5):
    steps = int(duration * rate)
    for _ in range(steps):
        subprocess.run(
            ["gz", "topic", "-t", "/cmd_vel", "-m", "gz.msgs.Twist",
             "-p", f"linear: {{x: {lx}}}, angular: {{z: {az}}}"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.0 / rate)

subprocess.run(["pkill", "-9", "-f", "gz sim"], stderr=subprocess.DEVNULL)
time.sleep(1)

print("Starting Gazebo with SIH Mine World...", flush=True)
gz = subprocess.Popen(
    ["gz", "sim", "-s", "-r", WORLD],
    env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

try:
    time.sleep(4)
    p0 = get_pose()
    print(f"Initial Spawn Pose: pos=({p0[0]:.2f}, {p0[1]:.2f}, {p0[2]:.2f}) | yaw={p0[5]:.1f}°", flush=True)

    # Step 1: Drive along apron towards the ramp entrance (x: -60 -> ~ -5)
    print("\nStep 1: Driving along apron towards ramp junction...", flush=True)
    cmd_vel(lx=2.0, az=0.0, duration=7.0)
    p1 = get_pose()
    print(f"Reached: pos=({p1[0]:.2f}, {p1[1]:.2f}, {p1[2]:.2f}) | yaw={p1[5]:.1f}°", flush=True)

    # Step 2: Turn right to face south (-Y, yaw ~ -90°)
    print("\nStep 2: Steering right towards Ramp 01 entrance...", flush=True)
    cmd_vel(lx=0.5, az=-0.5, duration=3.0)
    p2 = get_pose()
    print(f"Facing: pos=({p2[0]:.2f}, {p2[1]:.2f}, {p2[2]:.2f}) | yaw={p2[5]:.1f}°", flush=True)

    # Step 3: Drive onto and down Ramp 01 towards Bench 2
    print("\nStep 3: Driving forward onto Ramp 01...", flush=True)
    cmd_vel(lx=1.5, az=0.0, duration=5.0)
    p3 = get_pose()
    print(f"Ramp Progress: pos=({p3[0]:.2f}, {p3[1]:.2f}, {p3[2]:.2f}) | pitch={p3[4]:.1f}°, roll={p3[3]:.1f}°", flush=True)

    print("\nRamp navigation test completed successfully.", flush=True)
finally:
    gz.terminate()
    gz.wait()
