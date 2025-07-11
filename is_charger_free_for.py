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
# executor.wait(30)  # Wait for the simulation to be ready
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

start_time = time.time()
while time.time() - start_time < 5:
        pass  # wait for 5 seconds to let the simulation start
semantic = SmartChargingSemantic(env,batteries)

print("Checking if the AAA charger is free:")
# semantic.is_charger_free_for(batteries['battery_AAA/user_joint_16/'])
print(semantic.is_charger_free_for(batteries['battery_AA/1']))
print("Checking if the AA charger is free:")
print(semantic.is_charger_free_for(batteries['battery_AAA/1']))


# gemo2='table2_top'

# print(f"Force on {gemo2}: ")

# force_on_gemo = env.get_force_on_geom(gemo2)
# print(f"Force on {gemo2}: {force_on_gemo}")

# gemo2='AAA_charger/bottom'
# gemo2='AA_charger/bottom_left'


# force_on_gemo = env.get_force_on_geom(gemo2)
# print(f"Force on {gemo2}: {force_on_gemo}")


# # gemo2='table1_top'
# env.render_with_timer()
# gemo2='AAA_charger/bottom'


# force_on_gemo = env.get_force_on_geom(gemo2)
# print(f"Force on {gemo2}: {force_on_gemo}")

# gemo1='battery_AAA/battery_body'
# normal_force = env.get_normal_force(gemo2, gemo1)
# print(f"Normal force between {gemo2} and {gemo1}: {normal_force}")

# gemo1='table2_top'
# print(f"Normal force between {gemo1} and {gemo2}: {env.get_normal_force(gemo1, gemo2)}")

# gemo1='table1_top'
# gemo2='AA_charger/bottom_left'
# print(f"Normal force between {gemo1} and {gemo2}: {env.get_normal_force(gemo1, gemo2)}")
# 
"""
executor.plan_and_move_to_xyz_facing_down("ur5e_2", [-0.7, -0.6, 0.15])
"""

executor.wait(1000)
start_time = time.time()

while time.time() - start_time < 8:
        continue  # wait for 5 seconds to let the simulation start


# time.sleep(300)  # Wait for 1 second to ensure the simulation is ready













