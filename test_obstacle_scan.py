import subprocess, time, os, math

env = os.environ.copy()
res = subprocess.run(["bash", "-c", "source /opt/ros/jazzy/setup.bash && env"], capture_output=True, text=True)
for l in res.stdout.splitlines():
    if "=" in l:
        k, v = l.split("=", 1)
        env[k] = v

env["GZ_SIM_RESOURCE_PATH"] = "/home/krish-pc/SIH_Mine_Safety/gazebo/models:" + env.get("GZ_SIM_RESOURCE_PATH", "")

subprocess.run(["pkill", "-9", "-f", "gz sim"], stderr=subprocess.DEVNULL)
subprocess.run(["pkill", "-9", "-f", "parameter_bridge"], stderr=subprocess.DEVNULL)
time.sleep(1)

gz = subprocess.Popen(["gz", "sim", "-s", "-r", "/home/krish-pc/SIH_Mine_Safety/gazebo/worlds/sih_mine_world.sdf"],
                      env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(4)

# Obstacle directly in front of the truck at x = -56.0, y = 87.5 (4.0m in front of cab)
sdf_box = """<?xml version="1.0" ?>
<sdf version="1.9">
<model name="test_obstacle_front">
  <static>true</static>
  <pose>-55 87.5 2.0 0 0 0</pose>
  <link name="link">
    <collision name="c"><geometry><box><size>2 2 4</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>2 2 4</size></box></geometry><material><ambient>1 0 0 1</ambient></material></visual>
  </link>
</model>
</sdf>"""

r_spawn = subprocess.run([
    "gz", "service", "-s", "/world/sih_mine_world/create",
    "--reqtype", "gz.msgs.EntityFactory",
    "--reptype", "gz.msgs.Boolean",
    "--timeout", "3000",
    "--req", f'sdf: "{sdf_box.replace(chr(10), " ")}"'
], env=env, capture_output=True, text=True)

print("Spawn result:", r_spawn.stdout.strip())
time.sleep(1)

br = subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
                      env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(3)

r_scan = subprocess.run(["ros2", "topic", "echo", "/scan", "--once"], env=env, capture_output=True, text=True, timeout=8)

br.terminate()
gz.terminate()
gz.wait()

ranges = []
for l in r_scan.stdout.splitlines():
    if l.strip().startswith("- "):
        try: ranges.append(float(l.strip()[2:]))
        except: pass

print(f"Total range readings: {len(ranges)}")
amin = -3.14159265
ainc = 0.017501909
for i, r in enumerate(ranges):
    if not math.isinf(r) and not math.isnan(r) and r > 0.01:
        deg = math.degrees(amin + i * ainc)
        print(f"  Index {i:3d}: deg={deg:+6.1f}° -> range={r:.2f} m")
