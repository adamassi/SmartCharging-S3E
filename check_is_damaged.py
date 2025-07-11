from sim_ur5.mujoco_env.sim_env import SimEnv
from math import pi
from sim_ur5.motion_planning.motion_executor import MotionExecutor
import time
from sim_ur5.mujoco_env.common.ur5e_fk import forward
from battery_class import *
from semantic_questions import SmartChargingSemantic





env = SimEnv()


"""
workspace_x_lims = [-0.9, -0.54]
workspace_y_lims = [-1.0, -0.55]
"""
# battery_position = [[-0.6, -0.8, 0.13],]

executor = MotionExecutor(env)
joint_names = env.get_all_joint_names()
print(joint_names)
executor.wait(30)  # Wait for the simulation to be ready
"""
battery_AA/user_joint_1/
battery_aa_tray_charger/user_joint_3/
battery_AAA/user_joint_16/
battery_aaa_tray_charger/user_joint_7/
battery_C/user_joint_18/
battery_D/user_joint_19/
"""
print("waiting for 1 second")
# create Battery instances for joints containing "battery" in their name
batteries = {}
for joint_name in joint_names:
    if "battery" in joint_name.lower():
        key= joint_name.split('-')[0]  # Get the key without the trailing slash
        print(key)
        battery = Battery(name=joint_name,env=env)
        batteries[key] = battery
        print(f"Created Battery instance for joint: {joint_name}")
batteries['battery_AAA/1'].start_charging()  # Start charging the AAA battery
start_time = time.time()
while time.time() - start_time < 1:
        pass  # wait for 5 seconds to let the simulation start
semantic = SmartChargingSemantic(env)

print("Checking if the AAA charger is damaged:")
batteries['battery_AAA/1'].check_charge_progress()  # Check the charging progress of the AAA battery
batteries['battery_AAA/1'].stop_charging()  # Stop charging the AAA battery


"""
executor.plan_and_move_to_xyz_facing_down("ur5e_2", [-0.7, -0.6, 0.15])
"""
# batteries['battery_AAA/1'].stop_charging()  # Stop charging the AAA battery
executor.wait(1000)
start_time = time.time()

while time.time() - start_time < 1:
        continue  # wait for 5 seconds to let the simulation start
print(f"time -start: {time.time() - start_time}")
batteries['battery_AAA/1'].discharge_battery()  # Discharge the AAA battery
# print("BBBBBBBBBBBBBBBBB")

time.sleep(300)  # Wait for 5 minutes to ensure the simulation is ready













