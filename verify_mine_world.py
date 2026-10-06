"""
Fixed mine world verification - Phase 4 turn sends angular velocity 
to cmd_vel topic, then waits enough time to observe yaw change.
"""
import subprocess, time, os, math

env = os.environ.copy()
env["GZ_SIM_RESOURCE_PATH"] = (
    "/home/krish-pc/SIH_Mine_Safety/gazebo/models:"
    + env.get("GZ_SIM_RESOURCE_PATH", "")
)
WORLD = "/home/krish-pc/SIH_Mine_Safety/gazebo/worlds/sih_mine_world.sdf"
WORLD_NAME = "sih_mine_world"
SPAWN_X, SPAWN_Y = -60.0, 87.5

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
                if "x:" in l and "position"      in lines[max(0,i-1)]: x  = float(l.split(":")[1])
                elif "y:" in l and "position"    in lines[max(0,i-2)]: y  = float(l.split(":")[1])
                elif "z:" in l and "position"    in lines[max(0,i-3)]: z  = float(l.split(":")[1])
                elif "x:" in l and "orientation" in lines[max(0,i-1)]: qx = float(l.split(":")[1])
                elif "y:" in l and "orientation" in lines[max(0,i-2)]: qy = float(l.split(":")[1])
                elif "z:" in l and "orientation" in lines[max(0,i-3)]: qz = float(l.split(":")[1])
                elif "w:" in l and "orientation" in lines[max(0,i-4)]: qw = float(l.split(":")[1])
            r, p, ya = q2e(qx, qy, qz, qw)
            return x, y, z, r, p, ya
    return None

def gz_pub(topic, msg_type, payload):
    subprocess.run(
        ["gz", "topic", "-t", topic, "-m", msg_type, "-p", payload],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

subprocess.run(["pkill", "-9", "-f", "gz sim"],   stderr=subprocess.DEVNULL)
time.sleep(1)

print("Starting Gazebo Sim Harmonic with SIH Mine World...", flush=True)
gz = subprocess.Popen(["gz", "sim", "-s", "-r", WORLD],
    env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

try:
    time.sleep(4)

    # ─ Phase 1: Stability ──────────────────────────────────────
    print("\n╔═══════════════════════════════════════════════════════╗")
    print("║  PHASE 1: Spawn stability on mine road apron – 5s    ║")
    print("╚═══════════════════════════════════════════════════════╝")
    for t in range(1, 6):
        time.sleep(1)
        p = get_pose()
        if p:
            x, y, z, r, pi, ya = p
            ok = abs(r) < 3 and abs(pi) < 3
            on_road = abs(x - SPAWN_X) < 5 and abs(y - SPAWN_Y) < 5
            print(f"  t={t}s  x={x:9.3f}  y={y:7.3f}  z={z:7.3f}"
                  f"  roll={r:5.2f}°  pitch={pi:5.2f}°"
                  f"  {'✓ STABLE' if ok else '✗ TIPPING'}"
                  f"  {'✓ ON ROAD' if on_road else '? position'}")
        else:
            print(f"  t={t}s  (no pose)")

    # ─ Phase 2: Forward ────────────────────────────────────────
    print("\n╔═══════════════════════════════════════════════════════╗")
    print("║  PHASE 2: Forward  /cmd_vel linear.x=1.5 m/s  (4 s) ║")
    print("╚═══════════════════════════════════════════════════════╝")
    p0 = get_pose()
    for _ in range(8):
        gz_pub("/cmd_vel", "gz.msgs.Twist", "linear: {x: 1.5}")
        time.sleep(0.5)
    p1 = get_pose()
    if p0 and p1:
        dx = p1[0]-p0[0]; dy = p1[1]-p0[1]
        dist = math.sqrt(dx*dx + dy*dy)
        print(f"  Before  x={p0[0]:.3f}  y={p0[1]:.3f}")
        print(f"  After   x={p1[0]:.3f}  y={p1[1]:.3f}  z={p1[2]:.3f}")
        print(f"  Travel  {dist:.2f} m  {'✓ MOVING' if dist > 1 else '✗ NOT MOVING'}")
        print(f"  roll={p1[3]:.2f}°  pitch={p1[4]:.2f}°  {'✓ UPRIGHT' if abs(p1[3])<3 and abs(p1[4])<3 else '✗ TIPPING'}")

    # ─ Phase 3: Odom ───────────────────────────────────────────
    print("\n╔═══════════════════════════════════════════════════════╗")
    print("║  PHASE 3: /odom                                      ║")
    print("╚═══════════════════════════════════════════════════════╝")
    res = subprocess.run(["gz","topic","-e","-t","/odom","-n","1"],
                         env=env, capture_output=True, text=True, timeout=5)
    vals = [l.strip() for l in res.stdout.splitlines() if "x:" in l or "y:" in l or "z:" in l]
    for v in vals[:6]:
        print(f"  {v}")
    print(f"  {'✓ /odom is publishing' if vals else '✗ no odom data'}")

    # ─ Phase 4: Turn  (send cmd repeatedly so it's not ignored) 
    print("\n╔═══════════════════════════════════════════════════════╗")
    print("║  PHASE 4: Turn  angular.z=0.5  (3 s)                ║")
    print("╚═══════════════════════════════════════════════════════╝")
    # Stop first, get heading
    gz_pub("/cmd_vel","gz.msgs.Twist","linear: {x: 0}, angular: {z: 0}")
    time.sleep(0.5)
    p_before = get_pose()
    for _ in range(6):
        gz_pub("/cmd_vel","gz.msgs.Twist","angular: {z: 0.5}")
        time.sleep(0.5)
    p_after = get_pose()
    if p_before and p_after:
        dyaw = p_after[5] - p_before[5]
        print(f"  yaw before: {p_before[5]:.2f}°   after: {p_after[5]:.2f}°   Δyaw={dyaw:.2f}°"
              f"  {'✓ TURNING' if abs(dyaw)>0.5 else '✗ NOT TURNING'}")
        print(f"  z={p_after[2]:.3f}  roll={p_after[3]:.2f}°  pitch={p_after[4]:.2f}°"
              f"  {'✓ UPRIGHT' if abs(p_after[3])<3 and abs(p_after[4])<3 else '✗ TIPPING'}")

    print("\n╔═══════════════════════════════════════════════════════╗")
    print("║  ALL PHASES COMPLETE                                 ║")
    print("╚═══════════════════════════════════════════════════════╝")
finally:
    gz.terminate()
    gz.wait()
