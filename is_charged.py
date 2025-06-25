from sim_ur5.mujoco_env.sim_env import SimEnv
from math import pi
from sim_ur5.motion_planning.motion_executor import MotionExecutor
import time
from sim_ur5.mujoco_env.common.ur5e_fk import forward
from battery_class import Battery  # Import the Battery class



env = SimEnv()


executor = MotionExecutor(env)
joint_names = env.get_all_joint_names()
"""
battery_AA/user_joint_1/
battery_aa_tray_charger/user_joint_3/
battery_AAA/user_joint_5/
battery_aaa_tray_charger/user_joint_7/
battery_C/user_joint_9/
battery_D/user_joint_11/
"""

# Create Battery instances for joints containing "battery" in their name
batteries = {}
for joint_name in joint_names:
    if "battery" in joint_name.lower():
        battery = Battery(name=joint_name)
        batteries[joint_name] = battery
        print(f"Created Battery instance for joint: {joint_name}")

print("waiting for 1 second")
start_time = time.time()
while time.time() - start_time < 1:
    pass  # wait for 1 second to let the simulation start

executor.wait(1000)

# Example: Print battery statuses
for joint_name, battery in batteries.items():
    print(f"{joint_name}: {battery}")













